#include <Eigen/Dense>
#include <iostream>
 
using namespace std;

Eigen::VectorXf arrayToVector(float *a, int dim){
    Eigen::VectorXf v(dim);
    for(int i=0;i<dim;i++) v(i) = a[i];
    return v;
}

int main()
{
  Eigen::Matrix4f m;
  m <<  1,1,1,0,
        0,3,1,2,
        2,3,1,0,
        1,0,2,1;

  float i = 0;
  cout << 5/i;
}