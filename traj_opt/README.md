# Traj_opt - A general trajectory optimization library


## Author

Siyuan Wu

## File Structure

```
traj_opt                              
├─ include                            
│  ├─ bernstein                       
│  │  └─ bernstein_utils.hpp          
│  ├─ bspline                         
│  │  └─ bspline_optimizer.h          
│  ├─ polynomial                      
│  │  ├─ gcopter.hpp                  
│  │  ├─ mini_snap.h                  
│  │  ├─ poly_traj_optimizer.h        
│  │  └─ poly_traj_utils.hpp          
│  ├─ iosqp.hpp                       
│  ├─ lbfgs.hpp                       
│  ├─ lbfgs_robust.hpp                
│  └─ plan_container.hpp              
├─ launch                             
│  └─ test_minisnap.launch            
├─ rviz                               
│  └─ test_minisnap.rviz              
├─ src                                
│  ├─ bspline_optimizer.cpp           
│  ├─ gradient_descent_optimizer.cpp  
│  ├─ mini_snap.cpp                   
│  └─ poly_traj_optimizer.cpp         
├─ test                               
│  └─ test_minisnap.cpp               
├─ CMakeLists.txt                     
├─ README.md                          
└─ package.xml                        

```

## Thanks
Thanks following predecessors for their open-source contributions
- Xin Zhou (B-spline)
- Boyu Zhou (B-spline opt)
- Zhepei Wang (GCOPTER)
- Jesus Tordersillas (MINVO, spline convertor)