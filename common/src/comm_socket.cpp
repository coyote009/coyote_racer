/*
  Copyright 2019 coyote009

  This file is part of coyote_racer.

  coyote_racer is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  coyote_racer is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with coyote_racer.  If not, see <http://www.gnu.org/licenses/>.

 */

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
//#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <string.h>
#include <vector>
#include <deque>
#include <string>

#include "comm_socket.h"

#define TCP_BUFSIZE 1024
#define UDP_BUFSIZE 65536

static int tcp_server_init( comm_data &comm, unsigned short port )
{
    struct sockaddr_in loc_addr;
    int status;


    memset( &loc_addr, 0, sizeof(loc_addr) );
    loc_addr.sin_addr.s_addr = htonl( INADDR_ANY );
    loc_addr.sin_port = htons( port );
    loc_addr.sin_family = AF_INET;


    comm.loc_socket = socket( AF_INET, SOCK_STREAM, 0 );

    // In order to avoid "Address already in use"
    // https://freebsd.sing.ne.jp/lang/c/14.html
    int yes = 1;
    if( setsockopt( comm.loc_socket, SOL_SOCKET, SO_REUSEADDR,
                    (const char *)&yes, sizeof(yes) ) < 0 )
    {
        perror( "setsockopt" );
        return 1;
    }

    status = bind( comm.loc_socket, (struct sockaddr *) &loc_addr, sizeof(loc_addr) );
    if( status < 0 )
    {
        perror( "bind" );
        return 1;
    }

    return 0;
}

static int tcp_accept( comm_data &comm )
{
    struct sockaddr_in rem_addr;
    unsigned int rem_addr_size = sizeof( rem_addr );
    int status;


    status = listen( comm.loc_socket, 1 );
    if( status < 0 )
    {
        perror( "listen" );
        return 1;
    }

    printf( "Waiting for connection...\n" );
    comm.socket = accept( comm.loc_socket, (struct sockaddr *) &rem_addr, &rem_addr_size );
    printf( "Connected from (%s)\n", inet_ntoa( rem_addr.sin_addr ) );

    return 0;
}

static int tcp_set_nb( comm_data &comm )
{
    int status;

    
    //status = fcntl( comm.loc_socket, F_SETFL, fcntl( comm.loc_socket, F_GETFL, 0 ) | O_NONBLOCK );
    //if( status < 0 )
    //{
    //    perror( "fcntl" );
    //    return 1;
    //}

    status = fcntl( comm.socket, F_SETFL, fcntl( comm.socket, F_GETFL, 0 ) | O_NONBLOCK );
    if( status < 0 )
    {
        perror( "fcntl" );
        return 1;
    }

    return 0;
}

static int tcp_client_init_connect( comm_data &comm, const char *server_ip, unsigned short port )
{
    struct sockaddr_in server_addr;


    memset( &server_addr, 0, sizeof(server_addr) );
    server_addr.sin_addr.s_addr = inet_addr( server_ip );
    server_addr.sin_port = htons( port );
    server_addr.sin_family = AF_INET;

    int ret;
    ret = socket( AF_INET, SOCK_STREAM, 0 );
    if( ret < 0 )
    {
        return ret;
    }
    else
    {
        comm.socket = ret;
    }

    printf( "Trying to connect (%s)\n", server_ip );
    ret = connect( comm.socket, (struct sockaddr *) &server_addr, sizeof(server_addr) );

    return ret;
}

static int tcp_wait_cmd( comm_data &comm, const char *cmd )
{
    int numrcv;


    while( 1 )
    {
        numrcv = recv( comm.socket, comm.buf.data(), TCP_BUFSIZE, 0 );
        if( (numrcv == -1) || (numrcv == 0)  )
        {
            if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
            {
                usleep( 100000 );
            }
            else
            {
                perror( "recv" );
                return 1;
            }
        }
        else if( strncmp( comm.buf.data(), cmd, strnlen(cmd,256) ) == 0 )
        {
            printf( "received %s command\n", cmd );
            break;
        }
    }

    return 0;
}

static int tcp_check_cmd( comm_data &comm )
{
    int numrcv;


    numrcv = recv( comm.socket, comm.buf.data(), TCP_BUFSIZE, 0 );
    if( (numrcv == -1) || (numrcv == 0)  )
    {
        if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
        {
            return 0;
        }
        else
        {
            //perror( "recv" );
            return -1;
        }
    }
    else
    {
        return numrcv;
    }
}

#if 0
static int tcp_send_cmd( comm_data &comm, const char *cmd )
{
    int num;


    while( 1 )
    {
        num = send( comm.socket, cmd, (strnlen(cmd,256) + 1), 0 );
        if( (num == -1) || (num == 0) )
        {
            if( ( errno == EAGAIN ) || ( errno == EWOULDBLOCK ) )
            {
                usleep( 100000 );
            }
            else
            {
                perror( "send" );
                return 1;
            }
        }
        else if( num == ((int)strnlen(cmd,256) + 1) )
        {
            printf( "sent %s command\n", cmd );
            break;
        }
    }

    return 0;
}
#else
static int tcp_send_cmd( comm_data &comm, const char *msg )
{
    int ret;
    ret = send( comm.socket, msg, strnlen(msg,256)+1, 0 );
    return ret;
}
#endif

static int tcp_send_data( comm_data &comm, const char *data, int num )
{
    int ret;
    ret = send( comm.socket, data, num, 0 );
    return ret;
}

static int tcp_close( comm_data &comm )
{
    int status;


    status = close( comm.loc_socket );
    if( status < 0 )
    {
        return 1;
    }

    return 0;
}

int comm_server_init_wait( comm_data &comm, std::string &client_addr )
{
    comm.buf.resize( TCP_BUFSIZE );
    
    if( tcp_server_init( comm, 9876 ) )
    {
        return 1;
    }

    if( tcp_accept( comm ) )
    {
        return 1;
    }

    if( tcp_wait_cmd( comm, "ADR" ) )
    {
        tcp_close( comm );
        return 1;
    }

    client_addr = comm.buf.data() + 4;

    if( tcp_set_nb( comm ) )
    {
        tcp_close( comm );
        return 1;
    }

    return 0;
}

int comm_client_init_connect( comm_data &comm, const std::string &server_ip,
                              const std::string &client_ip, unsigned short port )
{
    comm.buf.resize( TCP_BUFSIZE );
    
    int ret;
    ret = tcp_client_init_connect( comm, server_ip.c_str(), port );
    if( ret )
    {
        return ret;
    }

    strncpy( comm.buf.data(), "ADR", 4 );
    strncpy( (comm.buf.data() + 4), client_ip.c_str(), client_ip.size() + 1 );
    send( comm.socket, comm.buf.data(), 4 + client_ip.size() + 1, 0 );

    return 0;
}

int comm_check_cmd( comm_data &comm, const char *command )
{
    int numrcv = tcp_check_cmd( comm );
    if( numrcv > 0 )
    {
        int cmd_len = strnlen( command, 256 );
        if( strncmp( comm.buf.data(), command, cmd_len ) == 0 )
        {
            return numrcv;
        }
    }

    return 0;
}

int comm_check_cmds( comm_data &comm, const std::vector<std::string> &commands )
{
    if( tcp_check_cmd( comm ) > 0 )
    {
        for( int i = 0; i < commands.size(); i++ )
        {
            if( strncmp( comm.buf.data(), commands[i].c_str(),
                         strnlen( commands[i].c_str(), 256 ) ) == 0 )
            {
                return i;
            }
        }
    }

    return -1;
}

int comm_send_cmd( comm_data &comm, const char *cmd )
{
    return tcp_send_cmd( comm, cmd );
}

void comm_fina( comm_data &comm )
{
    sleep( 1 );
    tcp_close( comm );
}

int comm_receive_data( comm_data &comm, const std::vector<std::string> &commands,
                       std::vector<unsigned char> &data )
{
    int num_received = tcp_check_cmd( comm );
    if( num_received > 0 )
    {
        for( int i = 0; i < commands.size(); i++ )
        {
            int cmd_len = strnlen( commands[i].c_str(), 256 );
            if( strncmp( comm.buf.data(), commands[i].c_str(), cmd_len ) == 0 )
            {
                int data_len = num_received - cmd_len - 1;
                data.resize( data_len );
                memcpy( data.data(), comm.buf.data() + cmd_len + 1, data_len );
                
                return i; 
           }
        }
    }

    return -1;
}

int comm_send_data( comm_data &comm, const char *cmd, const std::vector<unsigned char> &data )
{
    int cmd_len = strnlen( cmd, 256 );
    int total_len = cmd_len + 1 + data.size();

    if( total_len > TCP_BUFSIZE )
    {
        return -1;
    }

    memcpy( comm.buf.data(), cmd, cmd_len + 1 );
    memcpy( comm.buf.data() + cmd_len + 1, data.data(), data.size() );

    return tcp_send_data( comm, comm.buf.data(), total_len ) - cmd_len - 1;
}

struct udp_data
{
    int socket;
    struct sockaddr_in sock_addr;
};

static int udp_sender_init( udp_data &udp, const char *ip_addr, unsigned int port )
{
    int status;

    
    memset( &udp.sock_addr, 0, sizeof(udp.sock_addr) );
    udp.sock_addr.sin_addr.s_addr = inet_addr( ip_addr );
    udp.sock_addr.sin_port = htons( port );
    udp.sock_addr.sin_family = AF_INET;

    udp.socket = socket( AF_INET, SOCK_DGRAM, 0 );

    status = fcntl( udp.socket, F_SETFL, fcntl( udp.socket, F_GETFL, 0 ) | O_NONBLOCK );
    if( status < 0 )
    {
        perror( "fcntl" );
        return 1;
    }

    return 0;
}

static int udp_write( udp_data &udp, std::vector<unsigned char> &data )
{
    int ret;

    
    ret = sendto( udp.socket, data.data(), data.size(), 0,
                  (struct sockaddr *) &udp.sock_addr, sizeof(udp.sock_addr) );
    if( ret == -1 )
    {
        //printf( "Error opening file: %s\n", strerror( errno ) );
        return 0;
    }

    return 0;
}

static void udp_close( udp_data &udp )
{
    close( udp.socket );
}

static int udp_receiver_init( data_receiver_data &dreceiver, unsigned short port )
{
    struct sockaddr_in recvSockAddr;
    int status;


    memset( &recvSockAddr, 0, sizeof(recvSockAddr) );
    recvSockAddr.sin_port = htons(port);
    recvSockAddr.sin_family = AF_INET;
    recvSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    dreceiver.socket = socket(AF_INET, SOCK_DGRAM, 0);

    status = bind( dreceiver.socket, (struct sockaddr *) &recvSockAddr, sizeof(recvSockAddr) );
    if( status < 0 )
    {
        perror( "bind" );
        return 1;
    }

    status = fcntl( dreceiver.socket, F_SETFL, fcntl( dreceiver.socket, F_GETFL, 0 ) | O_NONBLOCK );
    if( status < 0 )
    {
        perror( "fcntl" );
        return 1;
    }
    
    return 0;
}

static int udp_receive( data_receiver_data &dreceiver )
{
    int numrcv;
    numrcv = recvfrom( dreceiver.socket, dreceiver.buf.data(), UDP_BUFSIZE, 0, NULL, NULL);
    //if( numrcv < 0 )
    //{
    //    perror( "recvfrom" );
    //}
    
    return numrcv;
}

static void udp_close( data_receiver_data &dreceiver )
{
    close( dreceiver.socket );
}

static void *data_sender( void *arg )
{
    data_sender_data *dsender;
    udp_data udp;


    dsender = (data_sender_data *) arg;

    //if( udp_sender_init( udp, "192.168.11.103", 9877 ) )
    //if( udp_sender_init( udp, "192.168.11.38", 9877 ) )
    if( udp_sender_init( udp, dsender->ip_addr.c_str(), 9877 ) )
    {
        return 0;
    }

    while( !dsender->end )
    {
        std::vector<unsigned char> data;
        int sent = 0;

        
        pthread_mutex_lock( &dsender->mutex );

        if( dsender->data_list.size() )
        {
            data = dsender->data_list.front();
            dsender->data_list.pop_front();
        }
        
        pthread_mutex_unlock( &dsender->mutex );

        if( !data.empty() )
        {
            udp_write( udp, data );

            //printf( "sent data (%d)\n", data.size() );

            sent = 1;
        }

        if( sent == 0 )
        {
            usleep( 10000 );
        }
    }

    dsender->data_list.clear();
    dsender->end = 0;

    udp_close( udp );

    return 0;
}

int data_sender_init( data_sender_data &dsender, const std::string &ip_addr )
{
    dsender.end = 0;
    dsender.ip_addr = ip_addr;
    
    pthread_mutex_init( &dsender.mutex, NULL );

    int ret;
    ret = pthread_create( &dsender.thread, NULL, &data_sender, &dsender );

    return ret;
}

int data_sender_send( data_sender_data &dsender, const char *command,
                      const std::vector<unsigned char> &data )
{
    const int max_buffer_size = 3;
    const int max_udp_size = 65507; // https://qiita.com/tajima_taso/items/fdfed88c1e735ffb41e8

    if( strnlen( command, 256 ) > 3 )
    {
        return 1;
    }

    unsigned int num_data = data.size();
    if( num_data > (max_udp_size - 6) )
    {
        return 1;
    }

    std::vector<unsigned char> cmd_data( 6 );
    strncpy( (char *) cmd_data.data(), command, 4 );
    cmd_data[4] = num_data & 0xff;
    cmd_data[5] = (num_data & 0xff00) >> 8;

    cmd_data.insert( cmd_data.end(), data.begin(), data.end() );
    
    pthread_mutex_lock( &dsender.mutex );
    if( dsender.data_list.size() < max_buffer_size )
    {
        dsender.data_list.push_back( cmd_data );
    }
    pthread_mutex_unlock( &dsender.mutex );

    return 0;
}

int data_sender_fina( data_sender_data &dsender )
{
    dsender.end = 1;
    return pthread_join( dsender.thread, NULL );
}


int data_receiver_init( data_receiver_data &dreceiver )
{
    dreceiver.buf.resize( UDP_BUFSIZE );
    return udp_receiver_init( dreceiver, 9877 );
}

int data_receiver_receive( data_receiver_data &dreceiver, std::vector<unsigned char> &data )
{
    int numrcv;
    numrcv = udp_receive( dreceiver );

    int data_size = dreceiver.buf[4] | (dreceiver.buf[5] << 8);
        
    if( numrcv == (data_size + 6) )
    {
        dreceiver.cmd = (char *) dreceiver.buf.data();
        
        data.assign( &dreceiver.buf[6], &dreceiver.buf[numrcv] );
        //printf( "received data (%d)\n", numrcv );
        return 0;
    }

    return 1;
}

void data_receiver_fina( data_receiver_data &dreceiver, comm_data &comm )
{
    comm_send_cmd( comm, "EDD" );
    udp_close( dreceiver );
}
