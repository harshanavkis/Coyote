#ifndef MESSAGE_HPP
#define MESSAGE_HPP

struct msg {
    /*
     * Type of the message
     * 0 for Request (Client -> Server)
     * 1 for completion notification (Server -> Client)
     * 2 for failure notification (Server -> Client)
     */
    uint64_t type;

    /*
     * Only valid if type == 0.
     *
     * Size of the buffer to transfer
     */
    uint64_t size;

    /*
     * Only valid if type == 0.
     *
     * Direction of transfer
     * 0 for D2H
     * 1 for H2D
     */
    uint64_t direction;
};

#endif // MESSAGE_HPP
