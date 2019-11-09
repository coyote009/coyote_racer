#ifndef COMM_SOCKET_H_
#define COMM_SOCKET_H_

struct comm_data
{
    int loc_socket;
    int socket;
    std::vector<char> buf;
};

int comm_server_init_wait( comm_data &comm, std::string &client_addr );
int comm_client_init_connect( comm_data &comm, const std::string &server_ip,
                              const std::string &client_ip, unsigned short port );
int comm_check_cmd( comm_data &comm, const char *command );
int comm_check_cmds( comm_data &comm, const std::vector<std::string> &commands );
int comm_send_cmd( comm_data &comm, const char *cmd );
int comm_receive_data( comm_data &comm, const std::vector<std::string> &commands,
                       std::vector<unsigned char> &data );
int comm_send_data( comm_data &comm, const char *cmd,
                    const std::vector<unsigned char> &data );
void comm_fina( comm_data &comm );

struct data_sender_data
{
    pthread_t thread;
    pthread_mutex_t mutex;
    std::deque< std::vector<unsigned char> > data_list;
    int end;
    std::string ip_addr;
};

int data_sender_init( data_sender_data &dsender, const std::string &ip_addr );
int data_sender_send( data_sender_data &dsender, const char *command,
                      const std::vector<unsigned char> &data );
int data_sender_fina( data_sender_data &dsender );

struct data_receiver_data
{
    int socket;
    std::vector<unsigned char> buf;
    std::string cmd;
};

int data_receiver_init( data_receiver_data &dreceiver );
int data_receiver_receive( data_receiver_data &dreceiver, std::vector<unsigned char> &data );
void data_receiver_fina( data_receiver_data &dreceiver, comm_data &comm );

#endif
