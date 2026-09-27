#include "tram_model_core/model.hpp"
#include <iostream>
#include <iomanip>
int main(){tram_model_core::DualEkf filter;double t,dt,u,w;int valid;std::cout<<std::setprecision(16);while(std::cin>>t>>dt>>u>>w>>valid){auto r=filter.step(dt,u,w,valid,true);std::cout<<t<<" "<<r.state.x(1)<<" "<<r.state.x(0)<<"\n";} }
