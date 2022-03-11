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
#include <time.h>

#define MESSAGE_FREQ 1

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
private:
    char topic_message[256] = { 0 };
public:
    void callback(const std_msgs::String::ConstPtr& msg);
    void dist_callback(const std_msgs::Float32::ConstPtr& msg);
    char* getMessageValue();
    float time, d;
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    ROS_INFO("[client] I heard:[%s]", msg->data.c_str());
}

void Listener::dist_callback(const std_msgs::Float32::ConstPtr& dist) { 
    d = dist->data; // distance you want the robmobile to travel
    time = d*45; // time for which GO;50;50 should be published
}

char* Listener::getMessageValue() {
    return topic_message;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;
    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ
	Listener listener;
    ros::Subscriber client_sub = nh.subscribe("/client_messages", 1, &Listener::callback, &listener);
    ros::Subscriber dist_sub = nh.subscribe("/dist_messages", 1, &Listener::dist_callback, &listener);
    int sockfd, portno, n, choice = 1;
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
        error("[client] ERROR opening socket");
    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("[client] ERROR connecting");
    std::cout << "[client] How do you want the client to behave?:\n1. Be able to send messages manually\n2. Subscribe to /client_messages and send whatever's available there\nYour choice:";
    std::cin >> choice;

    ros::Time beginTime = ros::Time::now();

    while(ros::ok()){

        ros::spinOnce();
        std::cout << "[client] listener.time is: " << listener.time << std::endl;
        ros::Duration secondsIWantToSendMessagesFor = ros::Duration(listener.time);
        ros::Time endTime = beginTime + secondsIWantToSendMessagesFor;

        while(ros::Time::now() < endTime) {
            bzero(buffer,256);
            if (choice == 1) {
                printf("[client] Please enter the message: ");
                fgets(buffer,255,stdin);
            } else if (choice == 2) {
                strcpy(buffer, listener.getMessageValue());
                loop_rate.sleep();
            }
            int x;
                
            std::cout << "[client] endTime is: " << endTime << std::endl;
            x = 22 - strlen(buffer);   
            n = write(sockfd, buffer, strlen(buffer)+x);
            //std::cout << "[client] After write buffer is: " << buffer << std::endl;
            if (n < 0)
                error("[client] ERROR writing to socket");
                            
            if (echoMode) {
                printf("##########################\n");
                //printf("[client] Inside echo mode.\n");
                //bzero(buffer, 256);
                
                n = read(sockfd, buffer, strlen(buffer)+x);
                //std::cout << "[client] buffer length is: " << strlen(buffer) << std::endl;
                if (n < 0)
                    error("\n[client] ERROR reading reply");
                //printf("[client] buffer is: %s\n", buffer);
                std::cout << "[client] buffer is: " << buffer << std::endl;

                // separate the buffer
                std::istringstream ss(buffer); int result; std::vector<int> vect(6, 0); //3
                std::string token; int encoder_1, encoder_2;
                while(std::getline(ss, token, ';')) {
                    std::istringstream (token) >> result;  // convert string to integer
                    vect.push_back(result);
                    //std::cout << "[client] result: " << result << std::endl;
                }
                int size = vect.size(); 
                encoder_2 = vect.back();
                encoder_1 = vect.at(size-2);
                //std::cout << "[client] vect size: " << vect.size() << std::endl;
                std::cout << "[client] encoder 1: " << encoder_1 << std::endl;
                std::cout << "[client] encoder 2: " << encoder_2 << std::endl;
                std::cout << "[client] robmobile should run for " <<  listener.time << " seconds to travel " << listener.d << " meters." << std::endl;
                printf("##########################\n");
                ros::Duration(0.1).sleep(); // TODO: remove
            }
                //ros::spinOnce();
        }
    }
	    return 0;
}
