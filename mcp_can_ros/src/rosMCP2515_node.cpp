/**************************************************************************************************
 * @file        rosMCP2515_node.cpp
 * @author      Thomas
 * @brief       ROS node for the MCP2515 class
 **************************************************************************************************
  @ attention

  << To be Introduced >>

 *************************************************************************************************/
/**************************************************************************************************
 * How to use
 * ----------
 * This node is called "mcp2515_node"
 *   Configuration parameters:
 *      ~/config        /loop_rate
 *                      [ Floating data value, specifying the rate that the Linux device is
 *                      [ interrogated.
 *                      [ - Note, if none is provided will default to '4Hz'
 *
 *                      /address
 *                      [ The SPI address for the MAX6675 device
 *
 *   Publishers:
 *      None
 *
 *   Subscribers:
 *      None
 *
 *   Services:
 *      None
 *
 *************************************************************************************************/
// C System Header(s)
// ------------------
#include <thread>       // std::thread
#include <stdint.h>
#include <vector>       // for std::vector
#include <string>       // for strings

// C++ System Header(s)
// --------------------
// None

// Other Libraries
// ---------------
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "milibrary/BUSctrl.h"
#include "mcp_can_msgs/canMessage.h"
#include "std_msgs/UInt64.h"

// Project Libraries
// -----------------
#include "mcp_can_ros.h"
#include "mcp_can_dfs_ros.h"

//=================================================================================================
/**************************************************************************************************
 * Define any private variables
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
// None

/**************************************************************************************************
 * Define any private function prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *************************************************************************************************/
class rosMCP2515 {

private:
    std::string         kconfig_sub_area            = "config/";
    std::string         kcan_cfg1                   = "can/cfg1";
    std::string         kcan_cfg2                   = "can/cfg2";
    std::string         kcan_cfg3                   = "can/cfg3";
    std::string         kcan_gpio_interrupt_mask    = "gpio_interrupt_mask";

    std::string         knode_loop_rate             = "loop_rate";
    std::string         knode_spi_address           = "address";

    std::string         kMCP_publish_messages       = "can_messages";
    std::string         kMCP_subscribe_TX_request   = "request_tx";
    std::string         kMCP_subscribe_gpio_int     = "can_gpio_interrupt";

    // Inputs
    std::string         kSPI_transfer_service       = "transfer_spi";

private:
    MCP_CAN                 *_hardware_handle_  = NULL;
    uint32_t                _packet_sequence_                       =  0;

/**************************************************************************************************
 * == ROS STUFF == >>>   ROBOT OPERATING SYSTEM (ROS) OBJECTS    <<<
 *   -----------
 *  The follow are objects used for interfacing with the Robot Operating System.
 *************************************************************************************************/
    ros::NodeHandle         _nh_;
    ros::NodeHandle         _private_nh_;

    ros::NodeHandle         _nh_hardware_;
    ros::CallbackQueue      _hardware_callback_queue_;

    // PARAMETERS
    ////////////////////////
    double                          _loop_rate_parameter_;
    double                          _spi_address_;
    int                             _can_cfg_[3] = { 0 };
    int                             _gpio_interrupt_mask_;

    // MESSAGES
    ////////////////////////


    // PUBLISHERS
    ////////////////////////
    ros::Publisher                  _read_can_frame_publisher_;

    // SUBSCRIBERS
    ////////////////////////
    ros::Subscriber                 _request_tx_subscriber_;
    ros::Subscriber                 _gpio_interrupt_subscriber_;

    // TIMERS
    ////////////////////////
    ros::Timer                      _scheduled_read_timer_;

    // SERVICES
    ////////////////////////
    ros::ServiceClient              _spi_transfer_client_;

    // ACTIONS
    ////////////////////////

public:
/*
 *  @brief:  Create the MCP2515 node class, basic construction
 *
 *  @param:  Pointer to the 'NodeHandle'
 *  @param:  Pointer to the 'NodeHandle', setup for private - ("~")
 *  @retval: rosLnx class
 */
    rosMCP2515(ros::NodeHandle* normal, ros::NodeHandle* private_namespace):
    _nh_(*normal), _private_nh_(*private_namespace)
    {
        //_nh_hardware_.setCallbackQueue(&_hardware_callback_queue_);

        if (configNode() < 0) {
            ROS_ERROR("Error detected during MCP2515 construction, exiting node...");
            return;
        }

        nodeLoop();
    }

    /*
     *  @brief:  Separate function, to handle the hardware service callback queue.
     *           Intended to be used within a dedicated thread.
     *
     *  @param:  void
     *  @retval: void
     */
    void hardwareCallbackThread(void) {
        ros::SingleThreadedSpinner spinner;
        spinner.spin(&_hardware_callback_queue_);
    }

    /*
     *  @brief:  Function to encapsulate the looping of this node.
     *
     *  @param:  void
     *  @retval: void
     */
    void nodeLoop(void) {
        ROS_INFO("MCP2515 node ready for use");

        ros::Rate loop_rate(_loop_rate_parameter_);

        int result = _hardware_handle_->begin(MCP_ANY, (uint8_t) _can_cfg_[0],
                                                       (uint8_t) _can_cfg_[1],
                                                       (uint8_t) _can_cfg_[2]);

        if (result != CAN_OK) {
            ROS_ERROR("Failed to init MCP2515 (CAN bus) - %d", result);
        }

        if (_hardware_handle_->enOneShotTX() != CAN_OK) {
            ROS_ERROR("Didn't configure one shot mode correctly - %d", result);
        }

        if (_hardware_handle_->setMode(MCP_NORMAL) != MCP2515_OK) {
            ROS_ERROR("Couldn't configure the Mode to 'Normal' - %d", result);
        }

        ros::Duration(0.1).sleep();

        ros::spin();
    }

    /*
     *  @brief:  Setups the MCP2515 for the node, as per the expected input/configuration
     *           parameters from within the rosparam space.
     *           If there are any issues with the supplied values; which cannot be managed
     *           internally. Will return an error (value of -1), to be checked at the 'main' level
     *           to close the node down.
     *
     *  @param:  void
     *  @retval: Integer value - 0 = no issues, -1 = issues detected
     */
    int configNode(void)
    {
        //=========================================================================================
        // Input parameters
        _private_nh_.param<double>(kconfig_sub_area + knode_loop_rate,
                                   _loop_rate_parameter_,
                                   0.01);

        _private_nh_.param<int>   (kconfig_sub_area + kcan_gpio_interrupt_mask,
                                   _gpio_interrupt_mask_,
                                   1);

        _private_nh_.param<double>(kconfig_sub_area + knode_spi_address,
                                   _spi_address_,
                                   0.0);

        _private_nh_.param<int>   (kconfig_sub_area + kcan_cfg1,
                                   _can_cfg_[0],
                                   MCP_8MHz_1000kBPS_CFG1);

        _private_nh_.param<int>   (kconfig_sub_area + kcan_cfg2,
                                   _can_cfg_[1],
                                   MCP_8MHz_1000kBPS_CFG2);

        _private_nh_.param<int>   (kconfig_sub_area + kcan_cfg3,
                                   _can_cfg_[2],
                                   MCP_8MHz_1000kBPS_CFG3);

        for (uint8_t i = 0; i != 3; i++) {
            ROS_INFO("CAN configuration parameter cfg[%d] set to :: %d (0x%X)", i, _can_cfg_[i],
                                                                                   _can_cfg_[i]);

            if ( (_can_cfg_[i] < 0x00) || (_can_cfg_[i] > 0xFF) ) {
                ROS_ERROR("CAN cfg1...cfg3 parameters need to be 1byte wide; so a range of "
                          "0 to 255 is expected");
                return -1;
            }
        }

        //=========================================================================================
        // Duplication check

        //=========================================================================================

        _hardware_handle_ = new MCP_CAN(&_spi_transfer_client_, _spi_address_);

        ROS_INFO("MCP2515 has been setup");
        //=========================================================================================
        // Publishers
        _read_can_frame_publisher_ = _nh_.advertise<mcp_can_msgs::canMessage>(
                                                kMCP_publish_messages,
                                                20);

        _request_tx_subscriber_  = _nh_.subscribe(
                                                kMCP_subscribe_TX_request,
                                                20,
                                                &rosMCP2515::callbackTxReqestSubscriber,
                                                this);
        _gpio_interrupt_subscriber_ = _nh_.subscribe(
                                                kMCP_subscribe_gpio_int,
                                                20,
                                                &rosMCP2515::callbackGPIOInterruptSubscriber,
                                                this);

        //=========================================================================================
        // Timers
        _scheduled_read_timer_ = _nh_.createTimer(ros::Duration(1/_loop_rate_parameter_),
                                                  &rosMCP2515::callbackScheduledReadCheck,
                                                  this,
                                                  false);

        //=========================================================================================
        // Clients/Servers
        for (uint8_t i = 0; i != 10; i++) {
            if ( !(ros::service::exists(kSPI_transfer_service, false) ) )  {
                ROS_WARN("Required services are not currently available...pause count %d", i);
                ros::Duration(1).sleep(); // sleep for a second.
            }
            else {
                break;
            }

            // On last iteration
            if (i == 9) {
                ROS_ERROR("Timed out waiting for the services to be setup, shutdowning node...");
                return -1;
            }
        }

        _spi_transfer_client_   = _nh_.serviceClient<milibrary::BUSctrl>(
                                                kSPI_transfer_service,
                                                true);

        //=========================================================================================

        ROS_INFO("MCP2515 node constructed");
        return 0;   // If got to this point, no errors were detected
    }

    /*
     *  @brief:  Callback function for the subscription to read any new CAN message transmission
     *           requests
     *
     *  @param:  mistepper_msg OpenLoopReqt
     *  @retval: None
     */
    void callbackTxReqestSubscriber(const mcp_can_msgs::canMessage::ConstPtr& msg) {
        if ( (msg->data.size() == 0) || (msg->data.size() > 8) ) {return;};

        uint8_t resturn = _hardware_handle_->sendMsgBuf(msg->identifier,
                                                        msg->extendedMode,
                                                        msg->data.size(),
                                                        (uint8_t *)msg->data.data());

        ROS_INFO("Return state %d", resturn);
        ROS_INFO("Error counts -- TX : %d \t\t RX : %d", _hardware_handle_->errorCountTX(),
                                                         _hardware_handle_->errorCountRX());
        ROS_INFO("Error register -- : %d", _hardware_handle_->checkError());
    }

    /*
     *  @brief:  Wrapper function to read the CAN Received Buffers, and publish the contents
     *
     *  @param:  None
     *  @retval: None
     */
    void readMsgBuffandPub(void) {
        mcp_can_msgs::canMessage  pub;
        uint8_t rxBuff[8];
        uint8_t  len;
        unsigned long int rxId;
        uint8_t  rxExtd;

        _hardware_handle_->readMsgBuf(&rxId, &rxExtd, &len, rxBuff);

        pub.header.seq      =  _packet_sequence_++;
        pub.header.stamp    = ros::Time::now();
        pub.identifier      = (uint32_t) rxId;
        pub.extendedMode    = rxExtd;

        for (uint8_t i = 0; i != len; i++) {
            pub.data.push_back( rxBuff[i] );
        }

        _read_can_frame_publisher_.publish(pub);
    }

    /*
     *  @brief:  Callback function for the subscription to the GPIO interrupt pin.
     *
     *  @param:  mistepper_msg OpenLoopReqt
     *  @retval: None
     */
    void callbackGPIOInterruptSubscriber(const std_msgs::UInt64::ConstPtr& msg) {
        if ( ( ((uint64_t) msg->data ) & ( (uint64_t) _gpio_interrupt_mask_ )) == 0) {
            while(_hardware_handle_->checkReceive() == CAN_MSGAVAIL) {
                readMsgBuffandPub();
            }

        }
    }

    /*
     *  @brief:  Callback function to be used to see if there is any messages in the CAN buffer
     *           scheduled check
     *
     *  @param:  ROS Timer event
     *  @retval: None
     */
    void callbackScheduledReadCheck(const ros::TimerEvent& event) {
        while(_hardware_handle_->checkReceive() == CAN_MSGAVAIL) {
            readMsgBuffandPub();
        }
    }

    ~rosMCP2515() {
        ROS_INFO("Shutting down the node, and killing functions");
        delete _hardware_handle_;
    }

};

//=================================================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MCP2515_basic");
    ros::NodeHandle n;
    ros::NodeHandle private_params("~");

    rosMCP2515  node(&n, &private_params);

    ros::shutdown();            // If get to this point of the main() function. Then should be
                                // shutdowning the node - due to error
    ros::waitForShutdown();

    // On node shutdown, don't think it reaches this part of main()
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************
 * ===============================================================================================
 * Local functions
 * ===============================================================================================
 *************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
// None

