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

#define MESSAGE_FREQ 100
#define MESSAGE_SIZE 22

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
	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;
    ros::Rate loop_rate(MESSAGE_FREQ);
	Listener listener;
    ros::Subscriber client_sub = nh.subscribe("/cmd_motor", 1, &Listener::callback, &listener);
    ros::Publisher encoderTicks_pub = nh.advertise<std_msgs::Int32MultiArray>("encoder_ticks", 10);

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

    int encoder_1, encoder_2, encoder_1_prv, encoder_2_prv;
    encoder_1 = encoder_2 = encoder_1_prv = encoder_2_prv = 0;

    while(ros::ok()){

        bzero(buffer,256);
        strcpy(buffer, listener.getMessageValue());
        loop_rate.sleep();
        int x;
        x = MESSAGE_SIZE - strlen(buffer);
        std_msgs::Int32MultiArray enc_msgs;
        
        n = write(sockfd, buffer, strlen(buffer)+x);
        ROS_INFO("[client] Message recieved and written to server");

        if (n < 0)
            ROS_ERROR_THROTTLE(0.5, "[client] ERROR writing to socket");
                        
        if (echoMode) {
            n = read(sockfd, buffer, strlen(buffer)+x);
            if (n < 0)
                ROS_ERROR_THROTTLE(0.5, "\n[client] ERROR reading reply");
            ROS_DEBUG("[client] buffer is: %s", buffer);
            // separate the buffer
            std::istringstream ss(buffer); int result; std::vector<int> vect(6, 0);
            std::string token;
            while(std::getline(ss, token, ';')) {
                std::istringstream (token) >> result;  // convert string to integer
                vect.push_back(result);
            }
            int size = vect.size();
            encoder_1_prv = encoder_1;
            encoder_2_prv = encoder_2;
            encoder_2 = vect.back();
            encoder_1 = vect.at(size-2);
            ROS_DEBUG("[client] Left encoder: %d", encoder_1);
            ROS_DEBUG("[client] Right encoder: %d", encoder_2);          
            enc_msgs.data.push_back(encoder_2);
            enc_msgs.data.push_back(encoder_1);
        }        
      
        encoderTicks_pub.publish(enc_msgs);

        ros::spinOnce();

        enc_msgs.data.clear();
        //loop_rate.sleep();
    }
	    return 0;
}