#include <iostream>
#include <cstdint>
#include <cstring>

using namespace std;

typedef union{
  float f;
  char bytes[4];
} float_bytes;

int main(){

    cout << "Oi";

    int ss_state = 0;
    float_bytes trottle_rx;
    float_bytes steering_rx;
    float_bytes brake_rx;
    char hand_brake_rx;
    char manual_shift_rx;
    char reverse_rx;
    char gear_rx;

    // #Tf[0:3]Sf[0:3]Bf[0:3]HbRbGiMb$
    char msg[100];

    float_bytes t;
    t.f = 11.11;

    float_bytes s;
    s.f = 22.11;

    float_bytes b;
    b.f = 33.11;

    sprintf(msg, "#T%c%c%c%cS%c%c%c%cB%c%c%c%cH%cR%cG%cM%c$",
            t.bytes[0],t.bytes[1],t.bytes[2],t.bytes[3],
            s.bytes[0],s.bytes[1],s.bytes[2],s.bytes[3],
            b.bytes[0],b.bytes[1],b.bytes[2],b.bytes[3],
            1,1,23,1);

    for(int i = 0; i<strlen(msg); i++){

        switch (ss_state){
        case 0:
            if('#' == msg[i]){
                ss_state = 1;                 
            }
            break;
        
        case 1:
            switch (msg[i]){
                case 'T':
                    ss_state = 20;                 
                    break;

                case 'S':
                    ss_state = 30;                 
                    break;

                case 'B':
                    ss_state = 40;                 
                    break;

                case 'H':
                    ss_state = 50;                 
                    break;

                case 'M':
                    ss_state = 60;                 
                    break;

                case 'R':
                    ss_state = 70;                 
                    break;

                case 'G':
                    ss_state = 80;                                     
                    break;

                case '$':
                    ss_state = 0;          
                    break;
                
                default:
                    break;
            }
            break;        
        
        case 20:
            trottle_rx.bytes[0] = msg[i];
            ss_state = 21;                 
            break;        
        
        case 21:
            trottle_rx.bytes[1] = msg[i];
            ss_state = 22;                 
            break;        
        
        case 22:
            trottle_rx.bytes[2] = msg[i];
            ss_state = 23;                 
            break;        
        
        case 23:
            trottle_rx.bytes[3] = msg[i];
            ss_state = 1;                 
            break;        
        
        case 30:
            steering_rx.bytes[0] = msg[i];
            ss_state = 31;                 
            break;        
        
        case 31:
            steering_rx.bytes[1] = msg[i];
            ss_state = 32;                 
            break;
        
        
        case 32:
            steering_rx.bytes[2] = msg[i];
            ss_state = 33;                 
            break;        
        
        case 33:
            steering_rx.bytes[3] = msg[i];
            ss_state = 1;                 
            break;        
        
        case 40:
            brake_rx.bytes[0] = msg[i];
            ss_state = 41;                 
            break;        
        
        case 41:
            brake_rx.bytes[1] = msg[i];
            ss_state = 42;                 
            break;        
        
        case 42:
            brake_rx.bytes[2] = msg[i];
            ss_state = 43;                 
            break;        
        
        case 43:
            brake_rx.bytes[3] = msg[i];
            ss_state = 1;                 
            break;        
        
        case 50:
            hand_brake_rx = msg[i];
            ss_state = 1;                             
            break;        
        
        case 60:
            manual_shift_rx = msg[i];
            ss_state = 1;                            
            break;        
        
        case 70:
            reverse_rx = msg[i];
            ss_state = 1;                             
            break;        
        
        case 80:
            gear_rx = msg[i];
            ss_state = 1;                             
            break;
        
        default:
            break;
        }
    }

    cout << endl << msg << endl
         << trottle_rx.f << endl
         << brake_rx.f << endl
         << steering_rx.f << endl
         << (int)hand_brake_rx << endl
         << (int)gear_rx << endl
         << (int)reverse_rx << endl
         << (int)manual_shift_rx << endl;


}