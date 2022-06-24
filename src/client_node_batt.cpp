#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"

/* added */
#include <sstream> 
#include <iostream>
#include <string>
#include "std_msgs/Float32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32MultiArray.h"
#include <time.h>
#include <math.h>
#include <comm_tcp/BatteryState.h>
//#include <sensor_msgs/BatteryState.h>

#define MESSAGE_FREQ 100
#define MESSAGE_SIZE 51

using namespace std;

class Listener {
private:
    char topic_message[256] = { 0 };
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    char* getMessageValue();
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    ROS_INFO("[client] I heard:[%s]", msg->data.c_str());
}

char* Listener::getMessageValue() {
    return topic_message;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "client_node_batt");
	ros::NodeHandle nh;
    ros::Rate loop_rate(MESSAGE_FREQ);
	Listener listener;
    ros::Subscriber client_sub = nh.subscribe("/cmd_motor", 1, &Listener::callback, &listener);
    ros::Subscriber client_sub_1 = nh.subscribe("/motor_starten", 1, &Listener::callback, &listener);
    ros::Publisher encoderTicks_pub = nh.advertise<std_msgs::Int32MultiArray>("encoder_ticks", 10);
    ros::Publisher batteryData_pub = nh.advertise<comm_tcp::BatteryState>("battery", 1000);

    int sockfd, portno, n;
    struct sockaddr_in serv_addr, cl_addr;
    struct hostent *server;
    char buffer[256];
    bool echoMode = false;

    if (argc < 3) {
       fprintf(stderr,"Usage: $ rosrun comm_tcp client_node <hostname> <port> --arguments\nArguments:\n -e : Echo mode\n");
       exit(0);
    }
    if (argc > 3)
		if (strcmp(argv[3], "-e") == 0)
			echoMode = true;
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        ROS_ERROR_ONCE("[client] ERROR opening socket");
    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        ROS_ERROR_ONCE("[client] ERROR connecting");

    int encoder_1, encoder_2; 
    double full_cap, remaining_cap, current, voltage, percentage_cap; 
    encoder_1 = encoder_2 = 0;

    while(ros::ok()){

        bzero(buffer,256);
        strcpy(buffer, listener.getMessageValue());
        loop_rate.sleep();
        int x;
        x = MESSAGE_SIZE - strlen(buffer);
        std_msgs::Int32MultiArray enc_msgs;
        comm_tcp::BatteryState batt_msgs;

        double temp_d;
        n = write(sockfd, buffer, strlen(buffer)+x);
        //ROS_INFO("[client] Message recieved and written to server");

        if (n < 0)
            ROS_ERROR_THROTTLE(0.5, "[client] ERROR writing to socket");
                        
        if (echoMode) {
            n = read(sockfd, buffer, strlen(buffer)+x);
            if (n < 0)
                ROS_ERROR_THROTTLE(0.5, "\n[client] ERROR reading reply");
            ROS_INFO("[client] buffer is: %s", buffer);
            // separate the buffer
            std::istringstream ss(buffer); int result; std::vector<double> vect(16, 0);
            std::string token;
            while(std::getline(ss, token, ';')) {
                std::istringstream (token) >> result;  // convert string to integer
                temp_d = ::atof(token.c_str());        // convert string to double
                vect.push_back(temp_d);
            }
            int size = vect.size();
            double encoder_1_d = vect.at(size-7); 
            double encoder_2_d = vect.at(size-6);
            encoder_1 = (int)encoder_1_d;
            encoder_2 = (int)encoder_2_d;

            full_cap = vect.at(size-5);  
            remaining_cap = vect.at(size-4); 
            current = vect.at(size-3); 
            voltage = vect.at(size-2);

            voltage = voltage/1000;  // from mV to V
            current = current/1000;  // from mA to A

            ROS_INFO("[client] Left encoder: %d", encoder_1);
            ROS_INFO("[client] Right encoder: %d", encoder_2);
            ROS_INFO("[client] Full charge Cap (mAh): %f", full_cap);
            ROS_INFO("[client] Remaining charge Cap (mAh): %f", remaining_cap);

            /* Taking care of division by zero */

            if (remaining_cap > 0 and full_cap > 0)
            {
                percentage_cap = remaining_cap / full_cap;    // (float)remaining_cap / (float)full_cap
            }
            else if (remaining_cap <= 0 and full_cap <= 0)
            {
                percentage_cap = 0.0;
            }

            ROS_INFO("[client] Percentage: %f", percentage_cap);
            ROS_INFO("[client] voltage (mV): %f", voltage);    
            ROS_INFO("[client] current (mA): %f", current);
            ROS_INFO("[client] Temperature (C): %f", vect.back());  // %d for int : %f for float/double
            cout << " --------------------- " << endl;
                 
            enc_msgs.data.push_back(encoder_2);
            enc_msgs.data.push_back(encoder_1);

            batt_msgs.header.stamp = ros::Time::now();
            batt_msgs.present = true;
            batt_msgs.percentage = percentage_cap;
            batt_msgs.voltage = voltage; 
            batt_msgs.current = current;
            batt_msgs.charge = remaining_cap;
            batt_msgs.capacity = full_cap; // design_capacity        
            batteryData_pub.publish(batt_msgs);
        }        

        encoderTicks_pub.publish(enc_msgs);

        ros::spinOnce();

        enc_msgs.data.clear();
        //loop_rate.sleep();
    }
	    return 0;
}