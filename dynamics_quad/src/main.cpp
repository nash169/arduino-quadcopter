#include <iostream>
#include <cmath>
#include <ctime>
#include <time.h>

#include <dynplot.hh>
#include "simulation.hh"

int main(int argc, char const *argv[])
{
	const double pi = 3.1415926535897;

  arma::vec q,
            q0,
            q_desired(12, arma::fill::zeros),
            u(4, arma::fill::zeros);

  q0 = { 0.2, -0.2,    0,	 	// Angular Position (Body Attached)
           0,    0,    0, 	// Angular Velocity (Body Attached) -1,0,1
        0.04, 0.04, 0.01, 	// Linear Postion (Inertial Frame)
         0.1, -0.3,    0};	// Linear Velocity (Body Attached)

  q_desired(8) = 0.08;

  q = q0;

  double dynFreq = 1000, 
  	   dynStep = 1/dynFreq, 
  	   t = 0,
  	   simt = 10.0;
	
	Robobee bee(q, dynFreq);     // ROBOBEE
  Controller ctr(q_desired);    // Controller
    
  time_t timer;
  time (&timer);
  struct tm *timeInfo = localtime(&timer);

  char *fname;
  int dummy = asprintf(&fname, "sim_%d-%d-%d/", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);

  std::string folder(fname);
  boost::filesystem::path dir(folder);

  if(boost::filesystem::create_directory(dir)) {
      std::cout << "Success" << "\n";
  }

  Iomanager manager("BeeBrain/", folder);
  manager.SetStream("trials.dat", "out");

  bool ANIMATE = true, REC = false;
    
  int length = 800, height = 600;
	double frameRate = 0.01;

	Display* Frame = new Display("Quadcopter Filght Simulation", length, height);
	Camera *Cam = new Camera(glm::vec3(-30.0,0.2,0.2), glm::vec3(0,0,0), glm::vec3(0,1,0));
	Light *Lamp = new Light(glm::vec3(2,1,5));
	Shader *myShader = new Shader("../graphic/vertex.glsl", "../graphic/fragment.glsl");
	// Model *Base = new Model("../graphic/","base");
	Model *Robot = new Model("../graphic/","quad");

  if (!ANIMATE){
    delete Frame;
    delete Cam;
    delete Lamp;
    delete myShader;
    // delete Base;
    delete Robot;
  }

  if (REC)
    Frame->SetRecorder(folder + "flight.mp4");

  while (t < simt) {
  	// Real Time Robot Motion with 100Hz framerate
    if (ANIMATE && std::abs(remainder(t,frameRate)) < 0.00001){
      Frame->Clear(0.0f, 0.1f, 0.15f, 1.0f);
		  myShader->Bind();
		  Robot->SetPos( glm::vec3( q(7), q(8), q(6) ) );
		  Robot->SetRot( glm::vec3( q(1), q(2), q(0) ) );
		  myShader->Update(*Robot, *Cam, *Lamp);
		  Robot->Draw();
		  // myShader->Update(*Base, *Cam, *Lamp);
		  // Base->Draw();
		  Frame->Update();
    }

    // 1000Hz Classical Controller calculates 3 control torques
    u = arma::join_vert(ctr.AltitudeControl(q), ctr.DampingControl(q));
    q = bee.BeeDynamics(u);

  	t += dynStep;
  }

  if (ANIMATE){
    delete Frame;
    delete Cam;
    delete Lamp;
    delete myShader;
    // delete Base;
    delete Robot;
  }

	return 0;
}