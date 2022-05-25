# osqp_solver_sfunction
intergrate osqp solver into simulink using sfunction
## generate mex file command
```
mex caoxing_MPC_demo_SF.cpp mpc_osqp.cpp -Losqp/lib -losqp
```

## development progress
1. 编写运行test_osqp/helloworld.cpp，可以编译、运行和调试，证明C++运行环境没问题
2. 下载Eigen，放在如下位置：test_osqp/eigen3文件夹下面
3. 编写运行test_osqp/eigen_test.cpp，可以编译、运行和调试，证明调用Eigen没问题
4. 下载安装osqp库，放在如下位置：test_osqp/osqp文件夹下面
5. 按照教程，安装osqp库：https://blog.csdn.net/chen_mp/article/details/119465098，可以完成测试demo的运行，证明osqp安装成功