#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

int linear_z = 0;
int return_value  = 0;
uint16_t cmd_buffer[16] = {0};
uint16_t position_buffer[2] = {0};
uint16_t state_buffer[1] = {0};
int state = 0;


modbus_t *ctx;
int ID = 1;
int addr = 6144;
//int mode = 0;

int cmd_upper_reg = 1;
int cmd_lower_reg = 45529;

int position_upper_reg = 0;
int position_lower_reg = 0;

void orientalmotor_displace_callback(const std_msgs::Int32ConstPtr& msg){
    linear_z = msg->data;

    if(linear_z >= 65536){
        cmd_upper_reg = linear_z / 65536;
        cmd_lower_reg = linear_z - 65536;
    }
    else if (linear_z < 65536 && linear_z >= 0) {
        cmd_upper_reg = 0;
        cmd_lower_reg = linear_z;
    }
    else if (linear_z < 0) {
        cmd_upper_reg = ~0;
        cmd_lower_reg = linear_z;
    }
    printf("cmd_upper_reg: %d cmd_lower_reg: %d\n" , cmd_upper_reg ,cmd_lower_reg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orientalmotor_controller");
    ros::NodeHandle nh;

    ros::Subscriber orientalmotor_displace_sub = nh.subscribe("/orientalmotor/displace" , 5 , orientalmotor_displace_callback);
    ros::Publisher  orientalmotor_state_pub = nh.advertise<std_msgs::Int32>("/orientalmotor/state" , 5);

    std_msgs::Int32 motion_state_msg;

    ctx = modbus_new_rtu("/dev/Usb2Dynamixel_AL02L2KI" , 115200 , 'E' , 8 , 1 );
    modbus_set_slave(ctx , ID);

    if(modbus_connect(ctx) == -1){
        ROS_INFO("Connect failed");
        modbus_free(ctx);
        return -1;
    }

    return_value = modbus_write_register(ctx , 4065 , 1);//enable manually deceleration ratio option

    return_value = modbus_write_register(ctx , 897 , 1);//elecrical reducer A

    return_value = modbus_write_register(ctx , 899 , 1);//elecrical reducer B
    //elecrical deceleration ratio = 0.36 P/R

    return_value = modbus_write_register(ctx , 4033 , 2);//set unit 0  (step) to 2 (mm)

    return_value = modbus_write_register(ctx , 4035 , 5);//Ball screw Lead = 5 mm

    ros::Rate rate(10);
    while (ros::ok()){

        return_value = modbus_read_registers(ctx , 198 , 2 , position_buffer);
        //printf(" cmd_position Higher:%d lower:%d\n", position_buffer[0] , position_buffer[1]);
        position_upper_reg = position_buffer[0];
        position_lower_reg = position_buffer[1];
        printf("position upper reg:%d   position lower reg:%d\n" , position_upper_reg ,position_lower_reg);

        if(position_upper_reg > 0 ){
            motion_state_msg.data = 65536 * position_upper_reg + position_lower_reg;
        }
        else{
            motion_state_msg.data = position_lower_reg;
        }
        printf("positon state:%d\n" , motion_state_msg.data);
        orientalmotor_state_pub.publish(motion_state_msg);

        return_value = modbus_read_registers(ctx, 127 , 1 , state_buffer);
        int state = state_buffer[0];

//        for(int i = 0 ; i <1 ; i++){
//            printf("state: %d\n" , state_buffer[i]);
//        }


        if(linear_z != motion_state_msg.data){
                return_value = modbus_write_register(ctx , 125, 0);//clear operate reg

                return_value = modbus_write_register(ctx , addr , 0);
                return_value = modbus_write_register(ctx , addr+1 , 1);
                return_value = modbus_write_register(ctx , addr+2 , cmd_upper_reg);
                return_value = modbus_write_register(ctx , addr+3 , cmd_lower_reg);
                return_value = modbus_write_register(ctx , addr+4 , 0);
                return_value = modbus_write_register(ctx , addr+5 , 10000);// 10000
                return_value = modbus_write_register(ctx , addr+6 , 0);
                return_value = modbus_write_register(ctx , addr+7 , 2500); //  2500
                return_value = modbus_write_register(ctx , addr+8 , 0);
                return_value = modbus_write_register(ctx , addr+9 , 2500); //  2500
                return_value = modbus_write_register(ctx , addr+10 , 0);
                return_value = modbus_write_register(ctx , addr+11 , 1000);
                return_value = modbus_write_register(ctx , addr+12 , 0);
                return_value = modbus_write_register(ctx , addr+13 , 0);
                return_value = modbus_write_register(ctx , addr+14 , 0);
                return_value = modbus_write_register(ctx , addr+15 , 0);
                /*
                cmd_buffer[0]  = 0;                // mode of upper reg
                cmd_buffer[1]  = 1;                // mode of lower reg
                cmd_buffer[2]  = cmd_upper_reg;    // postion of upper reg
                printf("displace upper reg: %d\n" , cmd_buffer[2]);
                cmd_buffer[3]  = cmd_lower_reg;    // postion of lower reg
                printf("displace lower reg: %d\n" , cmd_buffer[3]);
                cmd_buffer[4]  = 0;                // velocitiy of upper reg
                cmd_buffer[5]  = 100;             // velocity of lower reg
                cmd_buffer[6]  = 0;                // acceleration of upper reg
                cmd_buffer[7]  = 100;              // acceleration of lower reg
                cmd_buffer[8]  = 0;                // deceleration of upper reg
                cmd_buffer[9]  = 100;              // deceleration of lower reg
                cmd_buffer[10] = 0;                // current of upper reg
                cmd_buffer[11] = 1000;             // current of lower reg
                cmd_buffer[12] = 0;
                cmd_buffer[13] = 0;
                cmd_buffer[14] = 0;
                cmd_buffer[15] = 0;

                return_value = modbus_write_registers(ctx , addr , 16 , cmd_buffer); //write No.0 (addr: 6144 ) data of motion
                printf("write OK %d\n" , return_value);
                */
                return_value = modbus_write_register(ctx , 125, 8); //start motion

        }



        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
