#include "MCL_localization.h"

using namespace std;

//PFLocalization类的构造函数
PFLocalization::PFLocalization(map_type* mapdata,vector<log_data>& logdata,ros::NodeHandle node)
{

    this->map_ = mapdata;					//	map
    this->log_data_ = logdata;				//odem and LiDAR dataset
    this->numParticles_ = PARTICLES_NUM;	// number of particles
    this->map_threshold_ = 0.95;			//map threshold to make sure when the random particles that are initially scatered randomly are inside the map
	this->node_ = node;						//ROS node coming from the Launch file
	this->resolution_ = 10;    				//map resolution 10cm
	this->lidar_offset_ = 25;				//LiDar offset 25cm
	this->lidar_range_max_ = 1000; 			//LiDAR max effictiveness range 1000cm
	this->ray_tracing_step_ = 1;			//ray tracing step size
	this->obstacle_threshold_ = 0.2;		//if the map probability is less than this value, it's considered an obstacle


	this->alpha1_ = 0.025;					//Odem motion model parameters- translation
	this->alpha2_ = 0.025;					//Odem motion model parameters- translation
	this->alpha3_ = 0.4;					//Odem motion model parameters- translation
	this->alpha4_ = 0.4;					//Odem motion model parameters- translation


	this->sigmahit_ = 2;					//variance of gaussian distribution in BeamModel
	this->lambdashort_ = 1.5;				//parameter of exponential distribution in BeamModel

	this->z_hit_ = 0.8;						// weight of Phi in BeamModel Gaussian distribution
	this->z_short_ = 0.2;					//weight of Pshort in BeamModel Exponential distribution
	this->z_rand_ = 0.0;					//weight of Prand in BeamModel Uniform distribution
	this->z_max_ = 0.0;					//weight of Pmax in BeamModel Uniform distribution

	//publis particle set pose
	publish_particlecloud_ = node_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
	lidar_path_pub = node_.advertise<nav_msgs::Path>("lidar_path", 1, true);
	lidar_odom_pub = node_.advertise<nav_msgs::Odometry>("lidar_odom", 1, true);
	this->particles_ros_.header.stamp = ros::Time::now();
	this->particles_ros_.header.frame_id = "map";
	this->particles_ros_.poses.resize(numParticles_);

	//publis robot pose
	publish_pose_ = node_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1, true);
	this->robot_ros_.header.stamp = ros::Time::now();
	this->robot_ros_.header.frame_id = "map";
}

//PFLocalization distructor <<< but why?
PFLocalization::~PFLocalization()
{
    
}


//Init Particles from data set function
void PFLocalization::InitParticles()
{
	int count = 1;
	
	while (count <= numParticles_) //generate random particles inside the map threshold
	{
		particle_state particle_temp;

		particle_temp.x =  rand() / (float)RAND_MAX * (map_->max_x - map_->min_x) + map_->min_x; //Init X Particles coordinates
		particle_temp.y = rand() / (float)RAND_MAX * (map_->max_y - map_->min_y) + map_->min_y;  //Init Y Particles coordinates

		if (map_->prob[(int) particle_temp.x][(int) particle_temp.y] <= map_threshold_)  //If the random particles are not within the map threshold try randomizing again
			continue;

		count++;	//count untill we get the number of effictive particles = total number of particles

		particle_temp.theta = rand() / (float)RAND_MAX * 2 * pi;  //initialize theta Randomly

		//make sure that the is betwwn -pi and pi 间
        if(particle_temp.theta > pi)
            particle_temp.theta -= 2 * pi;
        if(particle_temp.theta < -pi)
            particle_temp.theta += 2 * pi; 
		
		particle_temp.weight = 1.0/numParticles_;   // initialize parti weight to 1/NUM

		particles_.push_back(particle_temp);   //save to particle set
	}

	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;
	for(int i = 0; i < numParticles_; i++)
	{
		q = tf::createQuaternionMsgFromRollPitchYaw(0,0,particles_[i].theta);	// in rad units
		pose_ros.position.x = 0.1 * particles_[i].x;	//to change units from dm -> m
		pose_ros.position.y = 0.1 * particles_[i].y;
		pose_ros.position.z = 0.0;
		pose_ros.orientation = q;

		particles_ros_.poses[i] = pose_ros;
	}
	publish_particlecloud_.publish(particles_ros_);	  //Publish particles set topic
}


//Monte Carlo Localization Algorithm
void PFLocalization::MCLAlgorithm()
{
	particle_state particle_state_temp;
	vector<particle_state> particles_temp;
	particles_temp.resize(numParticles_);
	
	for (int i = 1;i < log_data_.size(); i++)	//loops through the data set (I Think LiDAR data and odem)
	{	
		//takes two adjacent frames in the dataset as two states of odem 
		motion_.x_front = log_data_[i-1].x_robot;
		motion_.y_front = log_data_[i-1].y_robot;
		motion_.theta_front = log_data_[i-1].theta_robot;
		motion_.x_rear = log_data_[i].x_robot;
		motion_.y_rear =log_data_[i].y_robot;
		motion_.theta_rear = log_data_[i].theta_robot;	

		for(int j = 0; j < numParticles_; j++)
		{
			particle_state_temp = SampleMotionModelOdometry(particles_[j]);	  //sampling Odem
			particles_temp[j] = particle_state_temp;
			
			// to edit original lidar scans
/* 			if(j>=2300 && j<=2400 || j>=2700 && j<=2800){	
				particles_temp[j] .x =150;
				particles_temp[j] .y =150;
				
			} */
		}

		// cout << "Reading (" << i << "/" << log_data_.size() << ") log data! ";
		// cout << "Robot Position (X,Y,theta): (" << log_data_[i].x_robot << "," << log_data_[i].y_robot << "," << log_data_[i].theta_robot << ")" << std::endl;

		particles_ = particles_temp;

		if(log_data_[i].data_type == ODOM_DATA)	  //check if the data is odometry and not LiDAR
		{
			Visualize();	//Dispaly the osem in real time in RVIZ

			continue;
		}
		else if(log_data_[i].data_type == LIDAR_DATA)	//if the frame data is LiDAR run this
		{
			measurement_.readings = log_data_[i].readings;   //store the LiDAR readings from the log file

			double total_weight = 0;
			for(int j = 0; j < numParticles_; j++)
			{
				float weight = MeasurementScoreModel(particles_[j]);	  //calculate the weight of each particle
				// cout << " each weight is : " << weight << endl;
				particles_[j].weight = weight;
				total_weight += particles_[j].weight;	  //sum of all particles weight
			}

			for(int j = 0; j < numParticles_; j++)
			{
				particles_[j].weight /= total_weight;  //normalize the weight of each particle by deviding by the total weight
				// avg_weight += particles_[j].weight/numParticles_;	
				// cout << " Normalized weight is : " << particles_[j].weight << endl;
			}

			float avg_weight = total_weight / numParticles_;
			// cout << " The average weight is : " << avg_weight << endl;

			LowVarianceSampler();		//Low Variance Resampling Algorithm

			CalRobotPose();		//Calculate the robot pose and publish the topic
			
			LidarOdomToPath();
		}
		else
		{
			cout << " ERROR：The Log Data Is Error!!!" << endl;
		}

		Visualize();	//display on RVIZ

		// sleep(1);		 //1s

		if(i <= 20)
			usleep(300000);   //前20帧数据延迟大一些，更清晰的观察粒子的重采样情况
		else
			usleep(40000); 
	}

}


//Probabilistic robotics book P103--- Based on the sampling algorithm in the odem motion model
particle_state PFLocalization::SampleMotionModelOdometry(particle_state particle)
{
	//Standard Odem Model
	float deltarot1 = atan2(motion_.y_rear - motion_.y_front,motion_.x_rear - motion_.x_front) - motion_.theta_rear; 	//Start rotation
	float deltatrans1 = sqrt(pow((motion_.x_rear - motion_.x_front),2) + pow((motion_.y_rear - motion_.y_front),2));	//translation rotation
	float deltarot2 = motion_.theta_rear - motion_.theta_front - deltarot1;																				//Start rotation

	float deltarot1_hat = deltarot1 - SampleStandardNormalDistribution(alpha1_*deltarot1 + alpha2_*deltatrans1);
	float deltatrans1_hat  = deltatrans1 - SampleStandardNormalDistribution(alpha3_*deltatrans1 + alpha4_*(deltarot1 + deltarot2));
	float deltarot2_hat  = deltarot2 - SampleStandardNormalDistribution(alpha1_*deltarot2 + alpha2_*deltatrans1);

	particle_state particle_temp;
	//the map for this example is in dm. the initial particles position is generated from the map and is also in dm, The odem is in cm, thus we need to convert it here
	particle_temp.x = particle.x + (deltatrans1_hat * cos(particle.theta + deltarot1_hat)) / resolution_;
	particle_temp.y = particle.y + (deltatrans1_hat * sin(particle.theta + deltarot1_hat)) / resolution_;
	particle_temp.theta = particle.theta + deltarot1_hat + deltarot2_hat;
	particle_temp.weight = particle.weight;

	return particle_temp;
}


//从标准正态分布中采样
float PFLocalization::SampleStandardNormalDistribution(float var)
{
	float sum = 0;
	for (int i = 0;i < 12; i++)
		//LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)))
		sum += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
	return (var / 6.0) * sum;
}



//计算高斯分布函数的概率
float PFLocalization::ProbMeasurementHit(float zkt_star, float zkt)
{
	if (zkt < 0 || zkt > (lidar_range_max_ / resolution_))
		return 0;
	else
	{
		float q;
		q = (1.0 / sqrt(2*pi*sigmahit_*sigmahit_)) * exp((-1/2*((zkt - zkt_star)*(zkt - zkt_star)))/(sigmahit_*sigmahit_));
		return q;	
	}
}


//计算指数分布函数的概率
float PFLocalization::ProbMeasurementShort(float zkt_star, float zkt)
{
	if(zkt < 0 || zkt < zkt_star)
		return 0;
	else
	{
		float q,eeta;
		eeta = 1 / (1 - exp(-1.0 * lambdashort_ * zkt_star));
		q = eeta * lambdashort_ * exp(-1.0 * lambdashort_ * zkt);
		return q;
	}
}


//计算均匀分布函数的概率
float PFLocalization::ProbMeasurementRandVal(float zkt)
{
	if(zkt < 0 || zkt >= (lidar_range_max_ / resolution_))
		return 0;
	else
		return 1.0 / (lidar_range_max_ / resolution_);
}


//计算是否为测量最大值的概率
float PFLocalization::ProbMeasurementMaxVal(float zkt)
{
	if(zkt == (lidar_range_max_ / resolution_))
		return 1;
	else
		return 0;
}


//根据得分模型计算每个粒子的权重
float PFLocalization::MeasurementScoreModel(particle_state particle)
{
	robot_state lidar_pose;
	float laser_end_x,laser_end_y,score = 0, zkt = 0;

    //计算激光雷达在map坐标系下的位姿
	lidar_pose.x = particle.x + (lidar_offset_ * cos(particle.theta)) / resolution_;	//(单位：dm)
	lidar_pose.y = particle.y + (lidar_offset_ * sin(particle.theta)) / resolution_;	//(单位：dm)
	lidar_pose.theta = particle.theta;

	//若雷达位姿在地图有效区域外,则终止此次计算,该粒子权重为0
	if(map_->prob[(int)lidar_pose.x][(int)lidar_pose.y] <= map_threshold_)   
		return 0.0;  
		
	for (int i = 0; i < LASER_BEAM_NUM; i++)
	{
		zkt = measurement_.readings[i];		//第i个激光束的测距，单位：dm,从log文件读取时就已经转换成dm了
		
		//若超出设置的lidar最大有效值，则此光束无效
		if (zkt > (lidar_range_max_ / resolution_))	  
			continue;

		//计算第i个激光束在世界坐标系下的角度
		float step_theta = ((double)i / 180.0) * pi + lidar_pose.theta - pi/2.0;

		laser_end_x = lidar_pose.x + zkt * cos(step_theta); 	//计算此激光束末端在map坐标系下的X坐标
		laser_end_y = lidar_pose.y + zkt * sin(step_theta); 	//计算此激光束末端在map坐标系下的Y坐标

		//若激光束末端在地图未知区域或无效区域，则跳过此次得分计算
		if(laser_end_x >= map_->max_x || laser_end_y >= map_->max_y || laser_end_x < map_->min_x || laser_end_y < map_->min_y 
		  															   || map_->prob[(int)laser_end_x][(int)laser_end_y] < 0)
		   continue;
		
		// if(map_->prob[(int)laser_end_x][(int)laser_end_y] >= 0 && map_->prob[(int)laser_end_x][(int)laser_end_y] < 0.15)
		// 	score++;

		score += map_->prob[(int)laser_end_x][(int)laser_end_y] < 0.15 ? 1 : 0; //累加，计算此帧lidar数据的得分
	}

	return score;	//返回当前帧lidar数据的得分，用此来表示粒子权重
}



//book P86---Low Variance Resampling Algorithm
void PFLocalization::LowVarianceSampler()
{
	vector<particle_state> particles_temp = particles_;
	float r = (rand() / (float)RAND_MAX) * (1.0 / (float)numParticles_); //初始化一个0~1之间的随机数
	float c = particles_[0].weight;
	int i = 0;

	for (int m = 0;m < numParticles_; m++)
	{
		float u = r + (float) m/ numParticles_; 		//移动随机数
		while (u > c && i < numParticles_ - 1)
		{ 
			i++;										//移动到下一个粒子
			c += particles_temp[i].weight;	
		}
		particles_[m] = particles_temp[i]; 	 			//复制被选择的粒子
		particles_[m].weight = 1.0 / numParticles_;		//重采样后粒子权重重置
		// cout << " each weight is : " << particles_[m].weight << endl;
	}	
}

int ctr;
//计算机器人位姿，并发布状态
void PFLocalization::CalRobotPose()
{
	float total_x = 0.0;
	float total_y = 0.0;
	float total_theta = 0.0;

	for(int i = 0; i < numParticles_; i++)
	{
		total_x += particles_[i].x;
		total_y += particles_[i].y;
		total_theta += particles_[i].theta;
	}	
	robot_pose_.x = total_x / numParticles_;
	robot_pose_.y = total_y / numParticles_;
	robot_pose_.theta = total_theta / numParticles_;

/* 	if(ctr>=350 && ctr<=360 || ctr>=430 && ctr<=440){
	robot_pose_.x =robot_pose_.x+40.0;
	robot_pose_.y = robot_pose_.y+40.0;
	}else if(ctr>=390 && ctr<=400){
	robot_pose_.x =robot_pose_.x-40.0;
	robot_pose_.y = robot_pose_.y-40.0;
	} */
/* 	cout << "Robot pose:  X: " << robot_pose_.x << ", Y: " << robot_pose_.y << ", Theta: " << robot_pose_.theta << ", counter: " << ctr <<  endl;
 */	ctr++;

	//显示机器人位姿
	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;
	q = tf::createQuaternionMsgFromRollPitchYaw(0,0,robot_pose_.theta);
	pose_ros.position.x = 0.1 * robot_pose_.x;	//单位：dm -> m
	pose_ros.position.y = 0.1 * robot_pose_.y;
	pose_ros.position.z = 0.0;
	pose_ros.orientation = q;
	robot_ros_.pose = pose_ros;

	publish_pose_.publish(robot_ros_);	//发布机器人状态


	static tf::TransformBroadcaster br;
  	tf::Transform transform;
 	transform.setOrigin( tf::Vector3(0.1 * robot_pose_.x, 0.1 * robot_pose_.y, 0.0) );
  	tf::Quaternion tf_q;
	tf_q.setRPY(0, 0, robot_pose_.theta);
  	transform.setRotation(tf_q);
 	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));	//发布base_link和map的tf关系

}


//显示实时更新的粒子集状态
void PFLocalization::Visualize()
{
	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;
	for(int i = 0; i < numParticles_; i++)
	{
		q = tf::createQuaternionMsgFromRollPitchYaw(0,0,particles_[i].theta);

		pose_ros.position.x = 0.1 * particles_[i].x;	//单位：dm -> m
		pose_ros.position.y = 0.1 * particles_[i].y;
	
		pose_ros.position.z = 0.0;
		pose_ros.orientation = q;

		if(ctr>=350 && ctr<=360 || ctr>=430 && ctr<=440){
		pose_ros.position.x+=4.0;
		pose_ros.position.y+=4.0;
		}else if(ctr>=390 && ctr<=400 ){
		pose_ros.position.x-=4.0;
		pose_ros.position.y-=4.0;
		}
	
		particles_ros_.poses[i] = pose_ros;
	}

	
	cout << "Robot pose:  X: " << pose_ros.position.x<< ", Y: " << pose_ros.position.y << ", Theta: " << ", counter: " << ctr <<  endl;
	publish_particlecloud_.publish(particles_ros_);	  //发布粒子集状态
}

void PFLocalization::LidarOdomToPath()
{
	geometry_msgs::Pose pose_ros;
	geometry_msgs::Quaternion q;

	float attx=150.0;
	float atty=150.0;
	for(int i = 0; i < numParticles_; i++)
	{
		q = tf::createQuaternionMsgFromRollPitchYaw(0,0,particles_[i].theta);
		pose_ros.position.x = 0.1 * particles_[i].x;	//单位：dm -> m
		pose_ros.position.y = 0.1 * particles_[i].y;
	}
	pose_ros.position.z = 0.0;
	//pose_ros.orientation = q;

	geometry_msgs::PoseStamped data;

	if(ctr>=350 && ctr<=360 || ctr>=430 && ctr<=440){
	data.pose.position.x=pose_ros.position.x+4.0;
	data.pose.position.y=pose_ros.position.y+4.0;
	}else if(ctr>=390 && ctr<=400){
	data.pose.position.x=pose_ros.position.x-4.0;
	data.pose.position.y=pose_ros.position.y-4.0;
	}else{
	data.pose.position.x=pose_ros.position.x;
	data.pose.position.y=pose_ros.position.y;
	}
	data.pose.position.z=pose_ros.position.z;
	data.pose.orientation.x=q.x;
	data.pose.orientation.y=q.y;
	data.pose.orientation.z=q.z;
	data.pose.orientation.w=q.w;

	lidar_odom.header.stamp = ros::Time::now();
	lidar_odom.header.frame_id = "map";
	lidar_odom.pose.pose.position.x=data.pose.position.x;
	lidar_odom.pose.pose.position.y=data.pose.position.y;
	lidar_odom.pose.pose.position.z=data.pose.position.z;
	lidar_odom_pub.publish(lidar_odom);


	lidar_path.header.frame_id="map";
	lidar_path.header.stamp=ros::Time::now();
	lidar_path.poses.push_back(data);
	lidar_path_pub.publish(lidar_path);
}

