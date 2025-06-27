/****************************************************************************/
/*	创建时间：2024-12-13														*/
/*	功能描述：																*/
/*				1. 通過定時器發送CAN數據										*/
/*				2. 通過獨立線程接收CAN數據									*/
/*				3. 記錄日志													*/
/*				4. 通過獨立線程接收決策指令									*/
/*				5. 根據發送來的目標區域經緯度及當前車輛經緯度計算方向盤轉角		*/
/****************************************************************************/

#include <string>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstdio>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <arpa/inet.h>   // for inet_ntop
#include <ctype.h>       // for toupper
#include <netinet/in.h>  // for sockaddr_in, htonl, htons, ntohs, INADDR_ANY

#include <GeographicLib/Geodesic.hpp> 
//#include <algorithm>  
#define _USE_MATH_DEFINES
#include <cmath>  
#include <iostream>
#include <vector>
#include <pthread.h>


using namespace std;

#define M_PI 3.14159265358979323846
 
#define NUM_THREADS 3

#define BUFSIZE 	800
#define SERV_PORT 	8000

unsigned char rev_data[8];
unsigned char send_data[8];

int rev_ok,rev_save_ok,rev_first,begin_run;

unsigned long	rev_count,frame_count;

double	_target_ind;

FILE* fp;

typedef struct can_input{
	unsigned char	gear;
	unsigned char   endSpeed;
	float			keepAngle;
}CAN_IN;


typedef struct decisionInput{
	double			endSpeed;
	double  		latitude;
	double  		longitude;
	double			headingAng;
}DECISION_IN;

typedef struct _vehicle_params {
    double 		condition_time_stamp;
    double 		gps_time_stamp;
    double 		vehicle_speed;
    float 		steer_angle;//deg
    unsigned 	steer_angle_speed; //deg/s
    float 		steer_cmd_callback;
    double 		engine_rpm;
    char 		target_gear;
    double 		acc_pos;
    double 		brake_press;
    char 		current_gear;//'0' '1' '2' '3' '4' '5' '6' 'P' 'N'
    double 		latitude;
    double 		longitude;
    double 		heading_angle;
}VPARAMS,*PVPARAMS;

VPARAMS cur_params;

vector<double> 	map_latitude_v;
vector<double>  map_longitude_v;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct udp_data{
	unsigned long 	id;
	double 			end_speed;
	double 			car_latitude;
	double	 		car_longitude;
	double			car_heading_angle;
	double			map_latitude[10];
	double			map_longitude[10];
} UDP_DATA;
 	
typedef struct _gis_info
{
    double latitude;
    double longitude;
    double heading_angle;
}GIS, *PGIS;

GIS map; 	

void handle_sigalrm(int sig) {
    // 定时信号处理函数
    printf("\nPeriodic task is running...\n\n");
	
	struct ifreq ifr = {0};
	struct sockaddr_can can_addr = {0};
	struct can_frame frame = {0};
	int sockfd = -1;
	int ret;
 
	/* 打开套接字 */
	sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(0 > sockfd) {
		perror("socket error");
		exit(EXIT_FAILURE);
	}
 
	/* 指定can0设备 */
	strcpy(ifr.ifr_name, "can0");
	ioctl(sockfd, SIOCGIFINDEX, &ifr);
	can_addr.can_family = AF_CAN;
	can_addr.can_ifindex = ifr.ifr_ifindex;
 
	/* 将can0与套接字进行绑定 */
	ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
	if (0 > ret) {
		perror("bind error");
		close(sockfd);
		exit(EXIT_FAILURE);
	}
 
	/* 设置过滤规则：不接受任何报文、仅发送数据 */
	setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
 
	/* 发送数据 */
	frame.data[0] = send_data[0];
	frame.data[1] = send_data[1];
	frame.data[2] = send_data[2];
	frame.data[3] = send_data[3];
	frame.data[4] = send_data[4];
	frame.data[5] = send_data[5];
	frame.data[6] = send_data[6];
	frame.data[7] = send_data[7];
	frame.can_dlc = 8;	//一次发送8个字节数据
	frame.can_id = 0x410;//帧ID为0x123,标准帧
 
	ret = write(sockfd, &frame, sizeof(frame)); //发送数据
	if(sizeof(frame) != ret) { //如果ret不等于帧长度，就说明发送失败
		printf("\nwrite can error!\n");
		//goto out;
	}
	else
	{
		printf("\nwrite can ret = %d.\n",ret);
	}
 
    close(sockfd);
}


    //0.角度计算calculate_delta_angle
    double calculate_delta_angle(double angle1, double angle2)
    {
        while (angle1 > 360) { angle1 -= 360; }
        while (angle1 < 0) { angle1 += 360; }
        while (angle2 > 360) { angle2 -= 360; }
        while (angle2 < 0) { angle2 += 360; }
        double delta = angle1 - angle2;
        if (delta > 180)
            {
            delta = delta - 360;
            }
        else if (delta < -180)
            {
            delta = 360 + delta;
            }
        return delta;
    }
    //3.航向角计算
    double Azimuth_withGeo(double lat1, double lon1, double lat2, double lon2)
    {
        double s12(0), azi1(0), azi2(0);
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
        cout << "geodesic lib:" << azi1 << endl;
        //cout << "c++:" << bearing(lat1, lon1, lat2, lon2) << endl;
        return azi1;
    }

    double calc_Lf(double v)
    {
        double k = 0.6, b = 0;
        double Ld = k * v + b;
        if (Ld < 1.49){
			Ld = 1.49; //考虑接收机和后轴之间的距离，由1.69减去1.2m 需要改动
		}

		//Ld = 2.85; //5kph
        //Ld = 5.7; //10kph 15kph
        return Ld;
    }
    
    //1.计算距离（起点与终点经纬度）
    double Distance_withGeo(double lat1, double lon1, double lat2, double lon2)
    {
        double s12(0);
        const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
        geod.Inverse(lat1, lon1, lat2, lon2, s12);
        return s12;
    }     
    
    int findWayPoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y)
    {
        int x_size = maps_x.size(), y_size = maps_y.size();
        int n = x_size <= y_size ? x_size : y_size;
        std::vector<double> disList(n, 0);
        for( int i = 0; i != n; ++i)
        {
            disList.at(i) = Distance_withGeo(x, y, maps_x.at(i), maps_y.at(i));
        }

        int closest = min_element(disList.begin(),disList.end()) - disList.begin();
        return closest;
    }
    
    //2.查找地图相关点    
	//cx, cy -- 路径经、纬度容器
    int calc_target_index(double lat, double lon, double Ld, vector<double> cx, vector<double> cy)
    {
		int ind = findWayPoint(lat, lon, cx, cy);
        cout << "最近点下标: " << ind << "/" << cx.size() << endl;
		double L = 0;
		while ((Ld > L) && ((ind + 1) < cx.size())) {
			L += Distance_withGeo(cx[ind + 1], cy[ind + 1], cx[ind], cy[ind]);
			ind += 1;
		}
		return ind;
    }      
    
    double obtainMapData()
	{
		double Ld;
        int ind;
        
		Ld = calc_Lf(cur_params.vehicle_speed / 3.6);
		ind = calc_target_index(cur_params.latitude, cur_params.longitude, Ld, map_latitude_v, map_longitude_v);  
		if (_target_ind >= ind) {
		    ind = _target_ind;
		}       

		
		if (ind < map_latitude_v.size()) {
			map.latitude = map_latitude_v[ind];
			map.longitude = map_longitude_v[ind];
			//map.heading_angle = map_heading_angle_v[ind];
		}
		else {
			map.latitude = map_latitude_v.back();
			map.longitude = map_longitude_v.back();
			//map.heading_angle = map_heading_angle_v.back();
			ind = map_latitude_v.size() - 1;
		}

        _target_ind = ind;
        return Ld;
	}

   

//vehicle_speed, heading_angle -- 当前速度、航向角
//lat,lon -- 当前坐标
//map_lat,lon -- 目标坐标
//map_heading_angle_v--目标航向角
//wheelbase--轴距
//预瞄距离（Ld）越长，控制效果会越平滑，预瞄距离越短，控制效果会越精确（同时也会带来一定的震荡）
double GPS2Steer(double vehicle_speed, double latitude, double longitude, double heading_angle,
               double map_latitude_v, double map_longitude_v, double wheelbase)
{
    double steer_value = 0;
	double Ld = 0;
 

    // 根据车速调整预瞄距离
    if (vehicle_speed < 0) {
        Ld = 1.60;  // 速度小于0时设定最小预瞄距离
    }
    else if (vehicle_speed >= 0 && vehicle_speed < 30) {
		Ld=calc_Lf(vehicle_speed);// 计算预瞄距离 Ld
        // Ld = 0.8;
    }
    else if (vehicle_speed >= 30 && vehicle_speed < 45) {
        Ld -= 7;  // 调整预瞄距离
    }
    else if (vehicle_speed >= 45 && vehicle_speed < 60) {
        Ld -= 10.5;  // 调整预瞄距离
    }


    double map_latitude = map_latitude_v;
    double map_longitude = map_longitude_v;


    // 计算纯跟踪控制的转角
    double alpha = calculate_delta_angle(Azimuth_withGeo(latitude, longitude, map_latitude, map_longitude), heading_angle);
    
    printf("\nalpha = %.2f\n",alpha);
    // 根据车速调整控制效果
    wheelbase=0.49 ;

    double delta =atan2(2.0 * wheelbase * sin(alpha  * M_PI / 180.0) / Ld, 1.0) * 180.0 / M_PI;
    
    printf("\ndelta = %.2f\n",delta);

    //计算航向角偏差
   // double heading_delta=map_heading_angle_v-heading_angle;
	// 限制转角范围
	double bound = 32.7;  // 前轮转向极限 需要根据实际调整
	if (delta > bound) {
		delta = bound;
	}
	if (delta < -bound) {
		delta = -bound;
	}

	//提前测量方向盘比例系数 （实际方向盘角度=计算出车身航向角偏差 * 比例系数） 比例系数即为方向盘打多少后车轮偏转1个单位度
	float Cal = 14.2813456 / 2;  //需要更新
	// 返回计算的方向盘转角（steer_value）
	steer_value = delta;
	if (steer_value < 0)
	{
		steer_value *= 1.068;
	}

	return steer_value;
}


 
void* can_receiving_thread(void* arg) {
    // 每20毫秒执行一次
    printf("\ncan_receiving_Thread [%lu] is running\n", pthread_self());
	struct ifreq ifr = {0};
	struct sockaddr_can can_addr = {0};
	struct can_frame frame = {0};
	int sockfd = -1;
	int i;
	int ret;
 
	/* 打开套接字 */
	sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(0 > sockfd) {
		perror("socket error");
		exit(EXIT_FAILURE);
	}
 
	/* 指定can0设备 */
	strcpy(ifr.ifr_name, "can0");
	ioctl(sockfd, SIOCGIFINDEX, &ifr);
	can_addr.can_family = AF_CAN;
	can_addr.can_ifindex = ifr.ifr_ifindex;
 
	/* 将can0与套接字进行绑定 */
	ret = bind(sockfd, (struct sockaddr *)&can_addr, sizeof(can_addr));
	if (0 > ret) {
		perror("bind error");
		close(sockfd);
		exit(EXIT_FAILURE);
	}
 
	/* 设置过滤规则 */
	//setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
	
	struct can_filter rfilter[1]; //定义一个 can_filter 结构体对象
	// 填充过滤规则，只接收 ID 为(can_id & can_mask)的报文
	rfilter[0].can_id = 0x405;
	rfilter[0].can_mask = 0x7FF;

	// 调用 setsockopt 设置过滤规则
	setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));	
 
	/* 接收数据 */
	for ( ; ; ) {
		if (0 > read(sockfd, &frame, sizeof(struct can_frame))) {
			perror("read error");
			break;
		}
 
 		printf("\nCAN_receive is running!\n");
		/* 校验是否接收到错误帧 */
		if (frame.can_id & CAN_ERR_FLAG) {
			printf("Error frame!\n");
			break;
		}
 
		/* 校验帧格式 */
		if (frame.can_id & CAN_EFF_FLAG)	//扩展帧
			printf("扩展帧 <0x%08x> ", frame.can_id & CAN_EFF_MASK);
		else		//标准帧
			printf("标准帧 <0x%03x> ", frame.can_id & CAN_SFF_MASK);
 
		/* 校验帧类型：数据帧还是远程帧 */
		if (frame.can_id & CAN_RTR_FLAG) {
			printf("remote request\n");
			continue;
		}
 
		/* 打印数据长度 */
		printf("[%d] [%lu] ", frame.can_dlc,frame_count++);
 
		/* 打印数据 */
		for (i = 0; i < frame.can_dlc; i++)
			printf("%02x ", frame.data[i]);
		printf("\n");
		
		for (i = 0; i < frame.can_dlc; i++)
			rev_data[i] = frame.data[i];
		rev_ok = 1;

	}
 
    return NULL;
}

void* udp_thread(void* arg) {

	printf("\nudp_Thread [%lu] is running\n", pthread_self());

    int sockfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_len;
    UDP_DATA recv_buf;
    int ret;
    char buffer[sizeof(UDP_DATA)];
 
    // 创建socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
 
    // 定义服务器地址
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(8000);
 
    // 绑定socket到地址
    bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
 
    client_len = sizeof(client_addr);
 
    // 接收数据
    while(1)
    {
    	ret = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &client_len);
    	if (ret > 0) {
        	// 将接收到的数据转换为结构体
        	memcpy(&recv_buf, buffer, sizeof(UDP_DATA));
 
 			pthread_mutex_lock(&mutex);
         	cur_params.latitude = recv_buf.car_latitude;
        	cur_params.longitude = recv_buf.car_longitude;
        	cur_params.heading_angle = recv_buf.car_heading_angle;

        	cur_params.vehicle_speed = rev_data[1];
        	map_latitude_v.clear();
        	map_longitude_v.clear();
        	
        	for(int i = 0; i < 10; i++)
        	{
        		map_latitude_v.push_back(recv_buf.map_latitude[i]);
        		map_longitude_v.push_back(recv_buf.map_longitude[i]);
        	}
        	_target_ind = 0;

 			pthread_mutex_unlock(&mutex);
        // 打印结构体数据
        	printf("Received ID: %ld\nend_speed: %.8f\ncar_latitude: %.8f\ncar_longitude: %.8f\ncar_heading_angle: %.8f\n\n", recv_buf.id, recv_buf.end_speed, recv_buf.car_latitude,recv_buf.car_longitude,recv_buf.car_heading_angle);
        	for(int i = 0; i < 10; i++)
        		printf("map_latitude[%d]: %.8f\nmap_longitude[%d]: %.8f\n",i,recv_buf.map_latitude[i], i, recv_buf.map_longitude[i]);
        	printf("\n\n");
        	begin_run = 1;
    	}
 	}
    // 关闭socket
    close(sockfd);
 
    return NULL;
}




void send_candata(CAN_IN& in, DECISION_IN& din, unsigned char * orders)
{
   // static unsigned char orders[8] = {0xff};
   
    *(orders+0) = 0x00;
    *(orders+1) = 0x00;
    *(orders+2) = in.gear;//1:D,2:R倒档
    *(orders+3) = 0;//0x31;

    *(orders+6) = in.endSpeed;
    *(orders+7) = 0xc8;

    int tmp = 0;
    float tmp1 = in.keepAngle;
    if(tmp1 <= 6) tmp1 *= 1.08;
    //20230418
    if(in.keepAngle < 0)//right-left && in.keepAngle >= -180    
    {
        tmp = (int)(1024 +  tmp1* 13);
        if(tmp < 581)
        {
            tmp = 581;
        }
    }
    else if(in.keepAngle >= 0)//left-right && in.keepAngle <=180
    {
        tmp = (int)(1024 + tmp1 * 15);
        if(tmp > 1515)
        {
            tmp = 1515;
        }
    }
    else
    {
        tmp = 1024;
    }

    if(in.gear == 2) tmp = 1024;///20240517

    *(orders+4)= (tmp >> 8) & 0x00ff;
    *(orders+5) = tmp & 0x00ff;

 //   return orders;
}



void* main_thread(void* arg) {

	printf("\nMain_Thread [%lu] is running\n", pthread_self());
 	
	
	CAN_IN 		can_vcu_in;
	DECISION_IN decision_control_in;
	
	can_vcu_in.keepAngle = 0.0;
	
	int add_sig = 1;

    while(1) {
 		double vehicle_speed, vehicle_latitude, vehicle_longitude, vehicle_heading_angle, map_latitude, map_longitude, wheelbase;
		double s12(0), azi1(0), azi2(0);
		double steer_error;

		pthread_mutex_lock(&mutex);
        	
        double Ld = obtainMapData();
        if (cur_params.vehicle_speed < 1)
		{
   			steer_error = 0;
		}
		
		wheelbase = 1.69;
 
 		steer_error = GPS2Steer(cur_params.vehicle_speed, cur_params.latitude, cur_params.longitude, cur_params.heading_angle, map.latitude, map.longitude, wheelbase);
 		
 		printf("\nsteer_error = %.2f\n",steer_error);

  		pthread_mutex_unlock(&mutex);
  		
		printf("\n\nStart Can \n\n");
        can_vcu_in.gear = 1;
        can_vcu_in.endSpeed = 0;
        
        if(add_sig ==1)
        {
        	can_vcu_in.keepAngle += 1.0;
        
        	if(can_vcu_in.keepAngle > 40.0)
        		add_sig = 0;
        }
        else
        {
        	can_vcu_in.keepAngle -= 1.0;
        
        	if(can_vcu_in.keepAngle < -40.0)
        		add_sig = 1;
        }        
        	
 		send_candata(can_vcu_in,decision_control_in,send_data);
        // 休眠20毫秒
        usleep(20000);
        
    }
 
    return NULL;
}




    string newFileName(int kid)
    {
        time_t t = time(0);
        char tmp[64];
        char env_home[1024];
        string output_path;

        //create the data path
        memset(env_home,0,sizeof(env_home));
        strcat(env_home,"/home/ztl/mainControl_Test");
        
        mkdir(strcat(env_home,"/data"),0777);
        mkdir(strcat(env_home,"/control"),0777);

        //generate the filename
        strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&t) ); 
        
        strcat(env_home,"/");

        strcat(env_home,tmp);

        sprintf(tmp,"_%d.txt",kid);
        output_path = strcat(env_home,tmp);
        
        return output_path;
    }

	void createFile(string output_path)
    {
        //若有文件在开启状态则关闭
		if (fp)
        {
            fclose(fp);
            fp = NULL;
        }
        //根据路径名称开启一个新的记录文件
        fp = fopen(output_path.data(),"w");

        if(fp == NULL)
        {
            printf("\ncreate file failed\n");
        }
		else
		{
			printf("\ncreate file success\n");
		}
		
		//将表头写入控制记录文件
        if(fp)
        {
            //changed 2020-03-03 lyu 
            string record = ""; 
            //时间组 
            record.append("time_stamp"); 				//时间戳
            record.append(",");
            record.append("time_stamp_decision"); 		//决策时间戳
            record.append(",");

            //gps组
            record.append("latitude");    				// 本車纬度
            record.append(",");
            record.append("longitude");    				// 本車经度
            record.append(",");
            record.append("map_latitude"); 				// 目標點纬度
            record.append(",");
            record.append("map_longitude");				// 目標點经度
            record.append(",");
            record.append("alpha");    					// alpha角
            record.append(",");
            record.append("steer_value");    			// 前輪轉角
            record.append(",");
            record.append("can_steer");    				// CAN線上的方向盤角度指令
            record.append(",");    
            record.append("speed_cmd");    				// CAN線上的速度指令
            record.append(",");
            record.append("vehicle_speed");    			// 車輛實際速度
            record.append(",");   
            record.append("vehicle_steer");    			// 車輛實際方向盤轉角
            record.append(",");                                      
                   
            record.append("\n");

            fputs(record.c_str(), fp);

			fclose(fp); 
        }
    }
 
int main() {
    pthread_t threads[NUM_THREADS];
    int rc;
    long t;
	string control_log_path;

	rev_ok = 0;
	rev_save_ok = 1;
	rev_first = 1;
	begin_run = 0;
	
	rev_count = 0;
	frame_count = 0;
	
	_target_ind = 0;
	
    control_log_path = newFileName(1);
	createFile(control_log_path);
	
	rc = pthread_create(&threads[0], NULL, udp_thread, (void *)t);
    if(rc){
          	printf("ERROR; return code from pthread_create() is %d\n", rc);
            exit(-1);
    }	
    
    while(begin_run == 0);

 
    for(t=1; t<NUM_THREADS; t++) {
        printf("Creating thread %ld\n", t);
		switch(t){
			case 1:
   				   	rc = pthread_create(&threads[t], NULL, can_receiving_thread, (void *)t);
					break;
			case 2:
   			     	rc = pthread_create(&threads[t], NULL, main_thread, (void *)t);
					break;
			/*case 3:
   			     	rc = pthread_create(&threads[t], NULL, udp_thread, (void *)t);
					break;
			case 4:
   			     	rc = pthread_create(&threads[t], NULL, thread_function3, (void *)t);
					break;*/
			default:
					break;
		}
        if (rc){
            printf("ERROR; return code from pthread_create() is %d\n", rc);
            exit(-1);
        }
    }

    struct itimerval tick;
    signal(SIGALRM, handle_sigalrm); // 设置信号处理函数
 
    // 初始化定时器
    tick.it_value.tv_sec = 0;
    tick.it_value.tv_usec = 100 * 1000; // 100毫秒
    tick.it_interval = tick.it_value; // 设置重复周期
 
    // 启动定时器
    if (setitimer(ITIMER_REAL, &tick, NULL) < 0) {
        perror("setitimer");
        return 1;
    }
 
 	
    // 等待所有线程完成
    for(t=0; t<NUM_THREADS; t++) {
        pthread_join(threads[t], NULL);
    }

	
    return 0;
}
