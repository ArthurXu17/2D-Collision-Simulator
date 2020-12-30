#include <iostream>
#include <cmath>
#include <fstream>
#define _USE_MATH_DEFINES

using namespace std;

class Vector{

    public:

    double x, y, angle, mag;

    Vector(){//default constructor
        x = 0;
        y = 0;
    }
    Vector(double a, double b, bool polar){//using basic vector knowledge to assign variables accordingly
        if (polar){
            mag = a;
            angle = b;
            x = a*cos(b);
            y = a*sin(b);
        }
        else{
            x = a;
            y = b;
            angle = atan2(b,a);
            mag = pow(a*a+b*b, 0.5);
        }       
    }
    //getters
    double xcom (){
        return x;
    }
    double ycom(){
        return y;
    }
    double magnitude(){
        return mag;
    }
    double getAngle(){
        return angle;
    }
    //setters
    void setx(double input){
        x = input;
    }
    void sety(double input){
        y = input;
    }
    //we only need the minus function and scalar multiple -> srel = s2-s1, A=F(1/m) and F=kx
    Vector minus(Vector v2){
        return Vector(x-v2.xcom(),y-v2.ycom(),false);
    } 
    Vector scalarMultiply (double scalar){
        return Vector(scalar*x,scalar*y,false);
    }
    //print out the vectors with barckets
    void print(){
        cout<<"("<<x<<","<<y<<") ";
    }
    void printpolar(){
        cout<<"("<<mag<<","<<angle*180/M_PI<<")";
    }
};

int main(){

    //all the data that is colllected
    double mass [2];
    double k [2];
    double r [2];
    double xi1,yi1,xi2,yi2,vxi1,vyi1,vxi2,vyi2;//translational
    double w1,w2,w1i,w2i,alpha1,alpha2,torque1,torque2,I1,I2;//rotational
    double friction = 0.064258;
    char outputstyle;
    Vector s1;
    Vector s2;
    Vector v1;
    Vector v2;

    cout<<"Enter mass of the first ball:\n";
    cin>>mass[0];
    cout<<"Enter mass of the second ball:\n";
    cin>>mass[1];
    cout<<"Enter spring constant of the first ball:\n";
    cin>>k[0];
    cout<<"Enter spring constant of the second ball:\n";
    cin>>k[1];
    cout<<"Enter radius of the first ball:\n";
    cin>>r[0];
    cout<<"Enter radius of the second ball:\n";
    cin>>r[1];
    cout<<"Enter initial x position of the first ball:\n";
    cin>>xi1;
    s1.setx(xi1);
    cout<<"Enter initial y position of the first ball:\n";
    cin>>yi1;
    s1.sety(yi1);
    cout<<"Enter initial x position of the second ball:\n";
    cin>>xi2;
    s2.setx(xi2);
    cout<<"Enter initial y position of the second ball:\n";
    cin>>yi2;
    s2.sety(yi2);
    cout<<"Enter initial x velocity of the first ball:\n";
    cin>>vxi1;
    v1.setx(vxi1);
    cout<<"Enter initial y velocity of the first ball:\n";
    cin>>vyi1;
    v1.sety(vyi1);
    cout<<"Enter initial x velocity of the second ball:\n";
    cin>>vxi2;
    v2.setx(vxi2);
    cout<<"Enter initial y velocity of the second ball:\n";
    cin>>vyi2;
    v2.sety(vyi2);
    cout<<"Enter the angular velocity of the first ball: \n";
    cin>>w1;
    cout<<"Enter the angular velocity of the second ball: \n";
    cin>>w2;
    cout<<"Output velocities in polar or cartesian (p/c):\n";
    cin>>outputstyle;
    
    ofstream output;
    output.open("Output.csv");
    if (outputstyle == 'c'){
        output<<"time,sx1,sy1,sx2,sy2,vx1,vy1,vx2,vy2,";
    }
    else if (outputstyle == 'p'){
        output<<"time,sx1,sy1,sx2,sy2,vmag1,vangle1,vmag2,vangle2,";
    }
    output<<"w1,w2,L1,L2,L total,Tr1,Tr2,Tr Total"<<endl;
    

    int steps = 20000;
    double time = 0;
    double dt = 0.0001;
    
    for(int i = 0; i < steps; i++){

        Vector srel2 = s2.minus(s1);//relative position of ball 2 relative to 1, simple subtraction
        Vector srel1 = s1.minus(s2);//relative position of ball 1 relative to 2, same concept
        Vector force1(0,0,false);//force applied to ball 1 is default 0
        Vector force2(0,0,false);//force applied to ball 2 is also default 0
        torque1 = 0;//likewise torque will also be 0
        torque2 = 0;
        I1 = 0.4*mass[0]*r[0]*r[0];//Rotational inertia of spheres
        I2 = 0.4*mass[1]*r[1]*r[1];
        //note by default, it means when there is no compression
        //the if statement below will change the force vectors if there is a compression

        //if the magnitude of the relative position is less than the radii of the balls combined, there is compression leading to spring force
        if (srel1.magnitude() < r[0] + r[1] ){
            Vector compression_distance2 = Vector(r[0] + r[1]-srel2.magnitude(),srel2.getAngle(),true);//the vector x in Hooke's law
            Vector compression_distance1 = Vector(r[0] + r[1]-srel1.magnitude(),srel1.getAngle(),true);
            double keff = (k[0]*k[1])/(k[0] + k[1]);//technically the balls act as 2 springs in series, this is the formula for the effective spring constant

            force1 = compression_distance1.scalarMultiply(keff);//applying f = kx
            force2 = compression_distance2.scalarMultiply(keff);

            if(w1i>w2i){
                /*torque = (net force) *(radius)
                         = (coefficient of friction)*(spring force)*radius*/
                torque1 = -force1.magnitude()*friction*compression_distance1.magnitude();
                torque2 = force2.magnitude()*friction*compression_distance2.magnitude();
            }
            else{
                torque1 = force1.magnitude()*friction*compression_distance1.magnitude();
                torque2 = -force2.magnitude()*friction*compression_distance2.magnitude();
            }

            // calculate rotational inertia for an ellipsoid: I = 0.2M(a^2 + b^2) where a and b are semi-minor axis
            I1 = 0.2 *mass[0]*(pow(r[0] - compression_distance1.magnitude(), 2) + pow(r[0], 2));
            I2 = 0.2 *mass[1]*(pow(r[1] - compression_distance2.magnitude(), 2) + pow(r[1], 2));
        }

        //caluculating a = f*(1/m) and angular acceleration = torque/rotational inertia
        Vector a1 = force1.scalarMultiply(1/mass[0]);
        Vector a2 = force2.scalarMultiply(1/mass[1]);

        alpha1 = torque1/I1;
        alpha2 = torque2/I2;

        //vf = vi+a*t
        v1.setx(v1.xcom()+a1.xcom()*dt);
        v1.sety(v1.ycom()+a1.ycom()*dt);
        v2.setx(v2.xcom()+a2.xcom()*dt);
        v2.sety(v2.ycom()+a2.ycom()*dt);
        //sf = si+v*t -> for small values of t    
        s1.setx(s1.xcom()+v1.xcom()*dt);
        s1.sety(s1.ycom()+v1.ycom()*dt);
        s2.setx(s2.xcom()+v2.xcom()*dt);
        s2.sety(s2.ycom()+v2.ycom()*dt);
        //wf=wi+alpha*t
        w1+=alpha1*dt;
        w2+=alpha2*dt;

        time = time + dt;

        if(outputstyle == 'c'){
            output<<time<<","<<s1.xcom()<<","<<s1.ycom()<<","<<s2.xcom()<<","<<s2.ycom()<<",";
            output<<v1.xcom()<<","<<v1.ycom()<<","<<v2.xcom()<<","<<v2.ycom()<<",";
        }
        else if (outputstyle == 'p'){
            output<<time<<","<<s1.xcom()<<","<<s1.ycom()<<","<<s2.xcom()<<","<<s2.ycom()<<",";
            output<<v1.magnitude()<<","<<v1.getAngle()<<","<<v2.magnitude()<<","<<v2.getAngle()<<",";
        }
        output<<w1<<","<<w2<<","<<w1*r[0]*mass[0]<<","<<w2*r[1]*mass[1]<<","<<w1*r[0]*mass[0]+w2*r[1]*mass[1]<<","<<0.5*I1*w1*w1<<","<<0.5*I2*w2*w2<<","<<0.5*I1*w1*w1+0.5*I2*w2*w2<<endl;

    }

    output.close();
}