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
#include <math.h>

#define MESSAGE_FREQ 1
#define STOP_DIST 4

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
    float diameter, s;
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    ROS_INFO("[client] I heard:[%s]", msg->data.c_str());
}

void Listener::dist_callback(const std_msgs::Float32::ConstPtr& angle) {
    float theta; 
    theta = angle->data; // angle in degree you want the robmobile to travel
    s = (diameter * theta * M_PI)/180; //length of an arc travel to rotate theta degree
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

    int encoder_1, encoder_2, encoder_1_prv, encoder_2_prv, encoder_1_diff, encoder_2_diff, counter;
    encoder_1 = encoder_2 = encoder_1_prv = encoder_2_prv = counter = 0;
    float total_distance_1,total_distance_2, distance_1, distance_2;
    total_distance_1 = total_distance_2 = 0;
    listener.diameter = 0.696;

    while(ros::ok()){

        bzero(buffer,256);
        if (choice == 1) {
            printf("[client] Please enter the message: ");
            fgets(buffer,255,stdin);
        } else if (choice == 2) {
            strcpy(buffer, listener.getMessageValue());
            loop_rate.sleep();
        }
        int x;
        x = 22 - strlen(buffer);   
        n = write(sockfd, buffer, strlen(buffer)+x);

        if (n < 0)
            error("[client] ERROR writing to socket");
                        
        if (echoMode) {
            printf("##########################\n");
            n = read(sockfd, buffer, strlen(buffer)+x);
            if (n < 0)
                error("\n[client] ERROR reading reply");
            //std::cout << "[client] buffer is: " << buffer << std::endl;
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
            //std::cout << "[client] encoder 1: " << encoder_1 << std::endl;
            //std::cout << "[client] encoder 2: " << encoder_2 << std::endl;
        }
    
        encoder_1_diff = encoder_1 - encoder_1_prv;
        encoder_2_diff = encoder_2 - encoder_2_prv;
    
        counter++;
        if (counter <= 2)
        {
            encoder_1_diff = 0.0;
            encoder_2_diff = 0.0;
        }
        
        distance_1 = (0.4911657 * encoder_1_diff)/145000;
        distance_2 = (0.4911657 * encoder_2_diff)/145000;
        total_distance_1 = distance_1 + total_distance_1;
        total_distance_2 = distance_2 + total_distance_2;

        std::cout << "[client] Wheel 1 distance is  (m): " << total_distance_1 << std::endl;
        std::cout << "[client] Wheel 2 distance is  (m): " << total_distance_2 << std::endl;
        std::cout << "[client] Wheel difference is (cm): " << (total_distance_1 - total_distance_2)*100 << std::endl;

        std::cout << "[client] listener.s is: " << listener.s << std::endl;

        if (abs(total_distance_1) >= STOP_DIST) //listener.s
        {
            n = write(sockfd, "GO;0;0", 6);
            std::cout << "[client] #### stoping the robot ####" << std::endl;
        }
        
        ros::spinOnce();
    }
	    return 0;
}
