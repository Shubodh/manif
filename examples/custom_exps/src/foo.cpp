#include <iostream>
#include <manif/manif.h>

using namespace manif;

template <typename Derived>
void print(const LieGroupBase<Derived>& g)
{
  std::cout << "Group degrees of freedom : " << g.DoF << std::endl;
//  std::cout << "Group underlying representation vector size : " << g.RepSize << std::endl;
  std::cout << "g's dim : " << g.Dim << std::endl;
  std::cout << "Current values : " << g << std::endl;
}

int main()
{
  SE2d p_2d;
  print(p_2d);

  SE3d p_3d;
  print(p_3d);
}
