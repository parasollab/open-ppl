#include "tcp_socket.h"

#include <chrono>
#include <cstring>
#include <iostream>

#ifdef _WIN32
 #define NOMINMAX
 #include <ws2tcpip.h>
 #include <ws2def.h>
 #include <windows.h>
 #include <winsock2.h>
 #include <io.h>
#else
 #include <netdb.h>
 #include <unistd.h>
#endif

using namespace std;


namespace nonstd {

  /*----------------------------- Construction -------------------------------*/

  tcp_socket::
  tcp_socket(const int _socket, const string& _server, const string& _port)
    : m_id(_socket), m_server(_server), m_port(_port)
  { }


  tcp_socket::
  tcp_socket() = default;


  tcp_socket::
  tcp_socket(const string& _server, const string& _port)
  {
    connect(_server, _port);
  }


  tcp_socket::
  ~tcp_socket()
  {
    disconnect();
  }

  /*------------------------- Connection Interface ---------------------------*/

  void
  tcp_socket::
  disconnect()
  {
    // Ensure there is a connection to close.
    if(m_id == -1)
      return;
    else if(s_debug)
      cout << "nonstd::tcp_socket: closing connection to " << m_server
           << " at port " << m_port << "." << endl;

    // Close listening thread.
    m_listening = false;
    if(m_listening_thread.joinable())
      m_listening_thread.join();

    // Close connection.
    close(m_id);
    m_id = -1;
    m_server = m_port = "";
  }


  bool
  tcp_socket::
  connect(const string& _server, const string& _port)
  {
    // Ensure the socket isn't already connected.
    if(m_id != -1) {
      if(s_debug)
        cout << "nonstd::tcp_socket: listen called when already connected.\n";
      return false;
    }
    else if(s_debug)
      cout << "nonstd::tcp_socket: initiating connection to " << _server
           << " at port " << _port << "..." << endl;

    // Setup helper data.
    struct addrinfo* info;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    // Open connection.
    try {
      // Get address info for communicating with server.
      if(getaddrinfo(_server.c_str(), _port.c_str(), &hints, &info) == -1)
        throw string("could not get address info");
      else if(s_debug)
        cout << "\tfound server info..." << endl;

      // Request a socket from the os.
      m_id = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
      if(m_id == -1)
        throw string("could not create socket");
      else if(s_debug)
        cout << "\tsocket created with id " << m_id
             << "..." << endl;

      // Connect.
      if(::connect(m_id, info->ai_addr, info->ai_addrlen) == -1)
        throw string("could not connect to server " + _server + " at port " +
            _port);
      else if(s_debug)
        cout << "\tsuccess. tcp_socket is connected." << endl;

      freeaddrinfo(info);
    }
    catch(string _s) {
      // Connection failed. Release any resources that have been acquired.
      cerr << "error: " << _s << "." << endl;
      freeaddrinfo(info);
      disconnect();
      return false;
    }

    // Connection succeeded, save server/port info.
    m_server = _server;
    m_port = _port;
    return true;
  }


  bool
  tcp_socket::
  listen(const string& _port, int _backlog, bool _concurrent)
  {
    // Ensure the socket isn't already connected.
    if(m_id != -1) {
      if(s_debug)
        cout << "nonstd::tcp_socket: listen called when already connected.\n";
      return false;
    }
    else if(s_debug)
      cout << "nonstd::tcp_socket: trying to listen on port " << _port << "...\n";

    // Setup helper data.
    struct addrinfo* info;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    // Open connection.
    try {
      // Get address info.
      if(getaddrinfo(nullptr, _port.c_str(), &hints, &info) == -1)
        throw string("could not get address info");
      else if(s_debug)
        cout << "\tgot address info..." << endl;

      // Request a listening socket from the os.
      m_id = socket(info->ai_family, info->ai_socktype, info->ai_protocol);
      if(m_id == -1)
        throw string("could not create socket");
      else if(s_debug)
        cout << "\tsocket created with id " << m_id << "..." << endl;

      // Bind the socket to the specified port.
      if(::bind(m_id, info->ai_addr, info->ai_addrlen) == -1)
        throw string("could not bind socket");
      else if(s_debug)
        cout << "\tbind successful..." << endl;

      // Listen.
      if(::listen(m_id, _backlog) == -1)
        throw string("could not set listen");
      else if(s_debug)
        cout << "success. tcp_socket is listening." << endl;

      freeaddrinfo(info);
    }
    catch(string _s) {
      // Listen failed. Release any resources that have been acquired.
      cerr << "error: " << _s << "." << endl;
      freeaddrinfo(info);
      disconnect();
      return false;
    }

    // Listening succeeded. Start listener in appropriate mode.
    m_listening = true;
    m_server = "self";
    m_port = _port;
    if(_concurrent)
      m_listening_thread = thread(listener, this);
    else
      listener(this);
    return true;
  }


  void
  tcp_socket::
  listener(tcp_socket* _t)
  {
    // Setup for polling the listening socket.
    fd_set listen_descriptors;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    while(true) {
      // Poll for connection request.
      do {
        this_thread::sleep_for(chrono::milliseconds(50));
        if(!_t->m_listening) return;
        FD_ZERO(&listen_descriptors);
        FD_SET(_t->m_id, &listen_descriptors);
      } while(select(_t->m_id + 1, &listen_descriptors, nullptr, nullptr,
            &timeout) < 1);

      // Receive incoming connection.
      struct sockaddr_storage info;
      socklen_t addr_length;
      int new_sock = accept(_t->m_id, (struct sockaddr*)&info, &addr_length);

      // If the connection fails, reset.
      if(new_sock == -1) {
        cerr << "nonstd::tcp_socket error: could not accept incoming request."
             << endl;
        continue;
      }

      // The connection succeeded. Get the connection info and call the handler
      // function.
      char ip[1024];
      char port[1024];
      getnameinfo((struct sockaddr*)&info, addr_length, ip, sizeof(ip),
          port, sizeof(port), NI_NUMERICSERV);

      if(s_debug)
        cout << "nonstd::tcp_socket: request accepted from " << ip << " at port "
             << port << "." << endl;

      // Call the handler function.
      _t->m_handler(tcp_socket(new_sock, string(ip), string(port)));
    }
  }

  /*--------------------------------------------------------------------------*/

}
