//
// Created by abiel on 12/9/21.
//

#ifndef FLOATSERIALIZATION_CANHANDLER_H
#define FLOATSERIALIZATION_CANHANDLER_H

#include <string>
#include <sys/epoll.h>
#include <unistd.h>
#include <condition_variable>
#include <queue>
#include <functional>
#include "socketcan.h"
#include "Vanttec_CANLib/CANMessage.h"

namespace vanttec {
    class CANHandler {
    public:
        CANHandler(const std::string &interfaceName);

        ~CANHandler();

        void register_parser(const std::function<void(uint8_t, can_frame)> &parser);

        void write(const vanttec::CANMessage &msg);

    public:
        void update_read();

        void update_write();

        std::vector<std::function<void(uint8_t, can_frame)>> msgParsers;

        std::queue<CANMessage> writeQueue;

        static const int MAX_EVENTS = 5;
        epoll_event evlist[MAX_EVENTS];
        int epfd{-1};
        int canfd{-1};

        std::condition_variable cv;
        std::mutex cv_m;
    };
}

#endif //FLOATSERIALIZATION_CANHANDLER_H
