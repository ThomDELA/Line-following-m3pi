#include "mbed.h"
#include "m3pi.h"

m3pi m3pi;                                  // Initialise the m3pi

Serial pc(USBTX, USBRX);                    // For debugging and pc messages, uses commented out to prevent hanging
BusOut myleds(LED1, LED2, LED3, LED4);

LocalFileSystem local("local"); 
Timer tm1;
Ticker tic1;

// all variables are float
#define D_TERMI 0
#define I_TERMOI 100
#define I_TERMI 100
#define P_TERMI 2000
#define MAXI 400
#define MINI -200
#define seuil(x) (x>400 ? 1 : 0)

float current_pos_of_line,derivative,proportional,power,integral,right,left,previous_pos_of_line;
int current_pos_of_linei,derivativei,proportionali,poweri,integrali,righti,lefti,previous_pos_of_linei;
float speed =0.2;
int speedi = 300;

unsigned short tabsensor[5];

// fonction permet de lire les capteurs sol et de convertir cela sous la forme d'un octet 
// seuls 5 bits sont utiles
// Vérifier l'ordre des bits sur la variable retrounée stat
// bit4 à bit0 de stat sachant que bit2 c'est le capteur milieu
unsigned char  lecture_captsol(unsigned short *tab){
    unsigned char stat=0;
    m3pi.calibrated_sensors(tab);
    for(unsigned short ii=0;ii<5;ii++){
            stat = (stat <<1)  | seuil(tab[ii]);
    }
    return stat;
}

void PIDi(){
    // Get the position of the line.
    current_pos_of_linei = 1000*m3pi.line_position();
    //v1 = tm1.read_us();
    proportionali = current_pos_of_linei;
    // Compute the derivative
    derivativei = current_pos_of_linei - previous_pos_of_linei;
    // Compute the integral
    integrali = (integrali+ I_TERMOI*proportionali)/(1+I_TERMOI);
    // Remember the last position.
    previous_pos_of_linei = current_pos_of_linei;
    // Compute the power
    poweri = (proportionali * (P_TERMI) ) + (integrali*(I_TERMI)) + (derivativei*(D_TERMI)) ;
    // Compute new speeds
    righti = speedi-(poweri*MAXI)/1000000;
    
    lefti = speedi+(poweri*MAXI)/1000000;
    // limit checks on motor control // a faire
    //MIN <right < MAX
    // MIN <left < MAX
    righti = (righti>MAXI ? MAXI :(righti<MINI ? MINI : righti));
    lefti = (lefti>MAXI ? MAXI :(lefti<MINI ? MINI : lefti));
    //right=0.0;
    //left=0.0;
    //v2=tm1.read_us();
    // send command to motors
    m3pi.left_motor(lefti/1000.);
    m3pi.right_motor(righti/1000.);
}

int count;
char id;
float trot[2]={speed,-speed};
char message[200];
int idx_mes=0;
unsigned char statcapt,lastState;
unsigned char indexLigne(0);
int nextDirection(0);
unsigned char bufferLigne[3];
unsigned char ligne;
unsigned char indexIntersection(0);
unsigned char intersections[200];
#define etatLigne(statcapt) (((statcapt&0x04)==0x04) ? 0x01 : 0) | (((statcapt&0x03)==0x03) ? 0x02 : 0)|(((statcapt&0x18)==0x18) ? 0x04 : 0)
#define majorite(x) (((x[1]&0x1+x[2]&0x01+x[2]&0x01)>1) ? 0x01 : 0x00)|(((x[1]&0x2+x[2]&0x02+x[2]&0x02)>1) ? 0x02 : 0x00)|(((x[1]&0x3+x[2]&0x03+x[2]&0x03)>1) ? 0x03 : 0x00)

void automf(){
    static unsigned char autom;
    char extremes;
    extremes = statcapt&0x11;

    //buffer Ligne prend la somme de :
    //0x01 si ligne au milieu
    //0x02 si ligne a droite
    //0x04 si ligne a gauche
    /*
    bufferLigne[indexLigne] = etatLigne(statcapt);
    indexLigne = (indexLigne + 1)%3;
    ligne = majorite(bufferLigne); //Vote a la majorite sur les trois dernieres mesures
    */
    ligne = (((statcapt&0x04)==0x04) ? 0x01 : 0) | (((statcapt&0x01)==0x01) ? 0x02 : 0)|(((statcapt&0x10)==0x10) ? 0x04 : 0);

    switch(autom){
        case 0: //do : Go forward till ligne aux extreme; exit : forced forward;
            myleds = 0x0F;
            if(statcapt!=0){ //Sur la ligne
                if(extremes!=0){ // Capteur gauche ou droite actif
                    indexIntersection++;
                    intersections[indexIntersection] = ligne;
                    m3pi.forward(0.2);
                    autom = 10;
                }
                else{
                    PIDi();
                }
            }
            else{       //Fin de ligne
                m3pi.stop();
                autom = 1;
                count = 0;
                m3pi.cls();
                message[idx_mes]='B';
                idx_mes++;
                message[idx_mes]=0;
                m3pi.locate(0,0);
                m3pi.print(message,idx_mes);
            }
            break;
            
        case 1: //Do : Wait 500ms; Exit :turn the robot
            myleds = 0x0F;
            count++;
            if(count>=10){
                m3pi.right(0.3);
                autom = 2;
            }
            break;
            
        case 2: // do : centrage sur la ligne; Exit : stop;
            myleds = 0x0F;
            if((statcapt&0x0E)!=0){
                count = 0;
                m3pi.stop();
                autom = 3;
            }
            break;
            
        case 3: //do : wait 500ms
            myleds = 0x0E;
            count++;
            if(count>=10){
                count = 0;
                autom=0;
            }
            break;
        
        case 10: //Entry : go forward; do : attend jusqu a depasser la ligne; 
                    //Exit : stop et decision du virage;
            myleds = 0x01;
            if(extremes==0){
                m3pi.stop();

                if(ligne&0x01==0x00){ //No forward in the intersection
                    intersections[indexIntersection] &= 0xFE;
                }
                
                if(intersections[indexIntersection]&0x02){ //Virage a droite
                    nextDirection = 1;
                    message[idx_mes] = 'R';
                    idx_mes++;
                    m3pi.right(speed);
                    autom =20;
                }
                else if(statcapt&0x04==0x04){ //Continue tout droit
                    nextDirection = 0;
                    message[idx_mes] = 'F';
                    idx_mes++;
                    autom = 0;
                }
                else if(intersections[indexIntersection]&0x04){ //Virage a gauche
                    nextDirection = 2;
                    message[idx_mes] = 'L';
                    idx_mes++;
                    m3pi.left(speed);
                    autom =2;
                }
                else
                    autom=100;
            }
            else{ //Ajoute chaque direction trouve sur le chemin
                intersections[indexIntersection] |= ligne;
                m3pi.forward(0.2);
                autom=10;
            }
            break;

        case 20: //Ignore forward line and go right
            myleds = 0x03;
            if (intersections[indexIntersection]&0x01){
                autom=21;
            }
            else{
                autom=2;
            }
            break;

        case 21 : //Do : Go through line; Exit : go to centering state
            myleds = 0x04;
            count++;
            if(count>=10){
                count =0;
                autom=2;
            }
            break;

        case 100:
            myleds=0x08;
            break;

        default:
            autom = 100;
            break;
    } 
}

volatile char flag10ms;
void inter1(){
        flag10ms=1;
}

int v;
unsigned char delai600ms;
char chain[10];
char flag1sec;

#define nMes_MAX 10
unsigned long tab[nMes_MAX];
unsigned short indMes(0) ;

int main() {
    m3pi.sensor_auto_calibrate();
    wait(1.);
    tic1.attach(&inter1,0.01);
    m3pi.cls();

    while(1) {
            lastState = statcapt;
            statcapt=lecture_captsol(tabsensor);

            if(flag10ms){
                flag10ms = 0;
                automf();
            } 
    }
}