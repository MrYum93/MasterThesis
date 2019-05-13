# use this to make and run the c code for the schedular. Both Encoder and Stepper code should be here.
#make
#sudo ./demo_encoder

# Alternatively use this line to compile without make
gcc -o demo_sched main.c  vectornav.c  -lrt -lm -I/usr/local/include -L/usr/local/lib
sudo ./demo_sched
