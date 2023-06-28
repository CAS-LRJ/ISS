#include<cmath>

struct PointCloud{  
  double x;
  double y;
  double z;  
  double stepsize;

  PointCloud(){
    this->x = 0.;
    this->y = 0.;
    this->z = 0.;    
  }

  PointCloud(double x, double y, double z, double stepsize){
    this->x = x;
    this->y = y;
    this->z = z;
    this->stepsize = stepsize;
  }  

// math.ceil(x[0] / stepsize), math.ceil(x[1] / stepsize), x[2]
  bool operator<(const PointCloud &pc) const {
    if (floor(x / stepsize) != floor(pc.x / stepsize)) {
        return x > pc.x;
    }
    else if (floor(y / stepsize) != floor(pc.y / stepsize)) {
        return y > pc.y;        
    }
    return z > pc.z;
  }
};