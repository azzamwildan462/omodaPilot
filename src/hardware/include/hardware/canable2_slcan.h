#ifndef CANABLE2_SLCAN_H
#define CANABLE2_SLCAN_H

#include <hardware/CANBUS_HAL.h>

class CANable2_SLCAN : public CANBUS_HAL
{
public:
    CANable2_SLCAN(HelpLogger *logger_ptr)
        : CANBUS_HAL(logger_ptr) {

          };

    int init(std::string device_name, int baudRate, int fd_baudrate = 2000000) override;

    int send_msg(can_frame_t *can_msg) override;
    can_frame_t recv_msg() override;
    int send_msgs(const std::vector<can_frame_t> &can_msgs) override;
    std::vector<can_frame_t> recv_msgs() override;

    int update() override;
    int init_update_as_new_thread() override;

    void shutdown() override;
};

#endif // CANABLE2_SLCAN_H