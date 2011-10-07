/* thread_socket_interface.cpp */

#include "thread_socket_interface.h"

UDPSocket receiver = UDPSocket();
UDPSocket sender;
string buf;
char* outputString = "HEAD%d,%.4f,%.4f,%.4f,%d,TAIL\nHEAD%d,%.4f,%.4f,%.4f,%d,TAIL";
vector<string> vectbuf, vect1, vect2;
int i,j;

void parse(string buf, vector<string> &vect) {
  int front, back;
  // get data segment
  front = buf.find("HEAD") + 6;
  back = buf.find("TAIL", front);
  string data = buf.substr(front, back-front-1);
  
  boost::split(vect, data, boost::is_any_of(","));
}

void connectionInit() {
  // Initialize sockets
  if(! (receiver.create())) {
		cout << "Error creating socket!" << endl;
		exit(-1);
	}
	if(! (receiver.bind(RECEIVE_PORT))) {
		cout << "Client: error connecting to ip: " << IP << "  port: " << RECEIVE_PORT << endl;
		exit(-1);
	}
  //receiver.set_non_blocking(true);
  receiver.set_timeout(0);             // using recv()    
  
  if(! (sender.create())) {
		cout << "Error creating socket!" << endl;
		exit(-1);
	}
	if(! (sender.bind(SEND_PORT))) {
		cout << "Client: error connecting to ip: " << IP << "  port: " << SEND_PORT << endl;
		exit(-1);
	}
	if(! (sender.setDestination(IP, SEND_PORT))) {
		cout << "error setting destination" << endl;
		exit(-1);
	}
	//sender.set_non_blocking(true);
	sender.set_timeout(0);    
}

bool getDeviceState (Vector3d& start_proxy_pos, Matrix3d& start_proxy_rot, bool start_proxybutton[], Vector3d& end_proxy_pos, Matrix3d& end_proxy_rot, bool end_proxybutton[]) {
  if (receiver.recv(buf)) {
    while (receiver.recv(buf));
    
    boost::split(vectbuf, buf, boost::is_any_of("\n"));
    parse(vectbuf[0], vect1);
    parse(vectbuf[1], vect2);

  	for(i=0; i<2; i++) {
      for(j=0; j<3; j++) {
          start_proxy_rot(j,i) = boost::lexical_cast<double>(vect1[(2-i)*4+j].c_str());
          end_proxy_rot(j,i) 	 = boost::lexical_cast<double>(vect2[(2-i)*4+j].c_str());
      }
  	}
  	for(j=0; j<3; j++) {
    	start_proxy_rot(j,i) = -1 * boost::lexical_cast<double>(vect1[(2-i)*4+j].c_str());
    	end_proxy_rot(j,i) 	 = -1 * boost::lexical_cast<double>(vect2[(2-i)*4+j].c_str());
    }
  	
  	// the three for loops above is equivalent to doing the following, but slightly more efficient.
  	/*
  	for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
          start_proxy_rot(j,i) = boost::lexical_cast<double>(vect1[i*4+j].c_str());
          end_proxy_rot(j,i) 	 = boost::lexical_cast<double>(vect2[i*4+j].c_str());
      }
  	}
  	
  	Matrix3d rot_ztonegx;
  	rot_ztonegx(0,2) = -1;
  	rot_ztonegx(1,1) = 1;
  	rot_ztonegx(2,0) = 1;
  	
  	start_proxy_rot = start_proxy_rot * rot_ztonegx;
  	end_proxy_rot = end_proxy_rot * rot_ztonegx;
  	*/
  	  	
 		start_proxy_pos(0) = 30 * boost::lexical_cast<double>(vect1[12].c_str());
    end_proxy_pos(0)   = 30 * boost::lexical_cast<double>(vect2[12].c_str());
    start_proxy_pos(1) = 30 * boost::lexical_cast<double>(vect1[13].c_str());
    end_proxy_pos(1)   = 30 * boost::lexical_cast<double>(vect2[13].c_str());
    start_proxy_pos(2) = 60 * boost::lexical_cast<double>(vect1[14].c_str()) - 6;
    end_proxy_pos(2)	 = 60 * boost::lexical_cast<double>(vect2[14].c_str()) - 6;
    
    start_proxybutton[0] = boost::lexical_cast<int>(vect1[16].c_str());
    start_proxybutton[1] = boost::lexical_cast<int>(vect1[17].c_str());
    end_proxybutton[0]  = boost::lexical_cast<int>(vect2[16].c_str());
    end_proxybutton[1]  = boost::lexical_cast<int>(vect2[17].c_str());
    
    return true;
  }
  return false;
}

void sendDeviceState (const Vector3d& start_feedback_pos, bool start_feedback_enabled, const Vector3d& end_feedback_pos, bool end_feedback_enabled) {
	char buf[1024];
	memset(&buf, 0, sizeof(buf));
	sprintf((char*)&buf, outputString, 0, start_feedback_pos(0)/30, start_feedback_pos(1)/30, (start_feedback_pos(2)+6)/60, start_feedback_enabled ? 1 : 0,
																		 1, end_feedback_pos(0)/30, 	end_feedback_pos(1)/30, 	(end_feedback_pos(2)+6)/60,  	end_feedback_enabled   ? 1 : 0);
	sender.send(string(buf));
}


void getDeviceState (double start_proxyxform[], bool start_proxybutton[], double end_proxyxform[], bool end_proxybutton[]) {
  if (receiver.recv(buf)) {
    while (receiver.recv(buf));
    
    boost::split(vectbuf, buf, boost::is_any_of("\n"));
    parse(vectbuf[0], vect1);
    parse(vectbuf[1], vect2);

		//sample configurator
		//double start_proxyxform[16] = {-0.1018,-0.9641,0.2454,0.0000,0.0806,0.2379,0.9680,0.0000,-0.9915,0.1183,0.0534,0.0000,-0.5611,-0.1957,-0.8401,1.0000};
		//double end_proxyxform[16] = {0.5982,0.7791,-0.1877,0.0000,0.3575,-0.0498,0.9326,0.0000,0.7172,-0.6250,-0.3083,0.0000,1.1068,0.2221,-0.7281,1.0000};

    // Extract transform matrix
    for (i=0; i<16; i++) {
        start_proxyxform[i] = boost::lexical_cast<double>(vect1[i].c_str());
        end_proxyxform[i]   = boost::lexical_cast<double>(vect2[i].c_str());
    }
    //cout << vect1[16] << "\t" << vect2[16] << endl;
    start_proxybutton[0] = boost::lexical_cast<int>(vect1[16].c_str());
    start_proxybutton[1] = boost::lexical_cast<int>(vect1[17].c_str());
    end_proxybutton[0]  = boost::lexical_cast<int>(vect2[16].c_str());
    end_proxybutton[1]  = boost::lexical_cast<int>(vect2[17].c_str());
  }
}

void sendDeviceState (double start_feedback_pos[], bool start_feedback_enabled, double end_feedback_pos[], bool end_feedback_enabled) {
	char buf[1024];
	memset(&buf, 0, sizeof(buf));
	sprintf((char*)&buf, outputString, 0, start_feedback_pos[0], start_feedback_pos[1], start_feedback_pos[2], start_feedback_enabled ? 1 : 0,
																		 1, end_feedback_pos[0],   end_feedback_pos[1],   end_feedback_pos[2],   end_feedback_enabled   ? 1 : 0);
	sender.send(string(buf));
}
