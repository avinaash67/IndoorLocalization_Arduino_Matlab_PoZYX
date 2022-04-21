clear;
clc;


%Load package for serial communication
pkg load instrument-control;


Serial_enable=1;
%Check for serial port is supported or not

BaudRate=115200;
Port="\\\\.\\COM3";
FIXED_POINT_FRACTIONAL_BITS =5;

recvValue=zeros(1,3,"int32");
dataPosInt32=[0 4 8 12 16];  %int32_t conversion
dataPosFloat16=[0 2 4 6 8 10 12 14 16 18];%float16_t conversion
dataPosInt16=[0 2 4 6 8 10 12];%int16_t conversion
startNum=2147483647;
stopNum=2130706431;
dataValByteCountInt32=16;  %[int32_t] Time=4 Xcoor=4 Ycoor=4 Zcoor=4 
dataValByteCountFloat16=18;  %[float16_t] Xacc=2 Yacc=2 Zacc=2 Xmag=2 Ymag=2 Zmag=2  Xgyro=2 Ygyro=2 Zgyro=2 
dataValByteCountInt16=12;%[int16_t] Xerr=2 Yerr=2 Zerr=2 XYerr=2 XZerr=2 YZerr=2 bytes

%Implementation  of function to show the inital settings of UART communication
%-------------------------------------------------------------------------------
function arduino = setSerialSettings(Port, BaudRate)
    arduino = serial(Port);
    set(arduino, "baudrate",BaudRate);
    printf(" - Port=%s\n",Port);
    printf(" - BaudRate=%d\n",BaudRate);
endfunction

%intial conditions for FIR filter 
%-------------------------------------------------------------------------------
%In this filter the taps are calculated automatincally by formula based on 
%the cut-off frequency, Sampling frequency, Stop band attenuation.

%cut off frequency = 1Hz
%stop band attenuation = 40dB at 2Hz
%sampling frequency(frequency of incomming data)= 16Hz

%fir function call syntax of octave
%-----------------------------------
%b = fir1 (n, w, type)
%This produces an order n FIR filter with the given frequency cutoff w,
%returning the n+1 filter coefficients in b. If w is a scalar, it specifies 
%the frequency cutoff for a lowpass or highpass filter.


f1 = 3; %cut off frequency
f2 = 4; % Pass band attenuation frequency 
delta_f = f2-f1; 
Fs = 16; %sampling frequency
dB  = 200; %attenuation needed
N = dB*Fs/(22*delta_f); %calculate the number of taps/co-efficients needed
N_val = round(N)-1; %rounding the value of N 

f =  [f1 ]/(Fs/2); %estimated cut off frequency calculated based on f1 and f2.
hc = fir1(N_val, f,'low'); %co-efficient calculation 

%Setting initial condition for the buffer
for i=1:1:N_val+1;
  buffer(1,i)= 0;
end  
global N_val = N_val;
global buffer = buffer;
global hc = hc; 


% Implementation of FIR filter
%-------------------------------------------------------------------------------
function [f_val] = fir_filt(value);
%getting global variables
global N_val
N_val=N_val;
global buffer 
buffer = buffer;
global hc;
hc = hc;

%setting the output variable to '0'
output = 0;
%shifting the array
for i=N_val+1:-1:2;
  buffer(1,i) = buffer(1,i-1);
endfor 

%buffer(1,0) is empty now
buffer(1,1) = value;

%multiplying with the coefficients of the filter with the buffer and summing 
for i=1:1:N_val+1;
  output = output + buffer(1,i) * hc(1,i);
endfor
f_val = output

endfunction  


%intial conditions for kalman filter
#-------------------------------------------------------------------------------
%Assigning initial conditions of states to 0;
global state_X=0;
global state_Y=0;
global state_Z=0;
global time_prev=0;
  
%State_Space modelling of kalman filter
global Xhat = [state_X;state_Y;state_Z];
  
%output matrix 
global Cmat = eye(3);
  
%estimated error covariance
global P=[10 0 0; 0 100 0; 0 0 200]
  
%kalman gain matrix Gk
global Gk=[0 0 0;0 0 0; 0 0 0];    %initially assaigned to Zero.
                                     %This later changes during the loop;  
%process noise covariance
global Q=[5 0 0;0 5 0;0 0 0.00001] ;
  
%Measurement noise covariance
global R = [40000 0 0;0 40000 0; 0 0 1000000000];%standard deviation = 100 mm ; variance = 10000; 


%intial conditions for DistanceCalc
#-------------------------------------------------------------------------------
%Assigning initial conditions of states to 0;
global state_distX=0;
global state_velX=0;

global state_distY=0;
global state_velY=0;

global state_distZ=0;
global state_velZ=0;



%Below implmentation change according to the initial conditions set
%--------------------------------------------------------------------
%kalman filter function implementation
#------------------------------------------------------------------------------
function [ret_posX ret_posY ret_posZ] = kalmanxyz(timeval,posX,posY,posZ,Xerr,Yerr,Zerr,XYerr,XZerr,YZerr);
  
%Getting global variables
%get global Xhat
global Xhat
Xhat1=Xhat;
%get global Pmatrix and Qmatrix
global P
P1=P;
global Q 
Q1=Q;
%get global Gk
global Gk
Gk1=Gk;
global Cmat
Cmat1=Cmat;

%global R
%R1=R;


%Setting Measur noise Covariance from POZYX
R1 = [single(Xerr) single(XYerr) single(XZerr);single(XYerr) single(Yerr) single(YZerr);single(XZerr) single(YZerr) single(Zerr)];  
  
%Measured values are got through this function and put into the Z variable
Z=[double(posX);double(posY);double(posZ)]; %This Z matrix does not contain A matrix because it is the real system from 
                                            %nature and we get the A mat along with the measured values.
  

Amat=[1 0 0; 0 1 0; 0 0 1];
  
%prediction :-
Xhat1 = (Amat*Xhat1);
 
predict_posX = Xhat1(1,1);
predict_posY = Xhat1(2,1);
predict_posZ = Xhat1(3,1);
  
P1 = Amat*(P1*Amat')+Q1;
  
%Updation
  
#update state variables matrix
Xhat1 = Xhat1+Gk1*(Z-(Cmat1*Xhat1));

#update state error covariance matrix
P1 = (eye(3)-(Gk1*Cmat1))*P1;
 
#update kalman gain matrix
Gk1 = P1*Cmat1'*pinv(P1*Cmat1'+R1);

ret_posX = predict_posX;
ret_posY = predict_posY;
ret_posZ = predict_posZ;
  
%update global variables
%get global Xhat
global Xhat
Xhat=Xhat1;
%get global Pmatrix and Qmatrix
global P
P=P1;
global Q 
Q=Q1;
%get global Gk
global Gk
Gk=Gk1;
global Cmat
Cmat=Cmat1;
global R
R=R1;

endfunction

%Implementation of function to calculate distance using accelerometer 
%-----------------------------------------------------------------
function[ret_dX ret_dY ret_dZ] = distanceCalc(time,Xacc1,Yacc1,Zacc1);

%getting global variables
%get global distanceX
global state_distX
distX=state_distX;
%get global velocityX
global state_velX
velX= state_velX;
%get global distanceY
global state_distY
distY=state_distY;
%get global velocityY
global state_velY
velY= state_velY;
%get global distanceZ
global state_distZ
distZ=state_distZ;
%get global velocityZ
global state_velZ
velZ= state_velZ;
%get global time_prev
global time_prev
time_prev1=time_prev;

%delta Time calculation and updation  :-
time_curr = time;
delta_time = time_curr - time_prev1;
time_prev1 = time_curr;

Amat2=[1 double(delta_time) double(power(delta_time,2)/2) 0 0 0 0 0 0 ;0 1 double(delta_time) 0 0 0 0 0 0 ; 0 0 1 0 0 0 0 0 0;0 0 0 1 double(delta_time) double(power(delta_time,2)/2) 0 0 0;0 0 0 0 1 double(delta_time) 0 0 0; 0 0 0 0 0 1 0 0 0;0 0 0 0 0 0 1 double(delta_time) double(power(delta_time,2)/2);0 0 0 0 0 0 0 1 double(delta_time);0 0 0 0 0 0 0 0 1];

X = [distX; velX; double(Xacc1);distY; velY; double(Yacc1);distZ; velZ; double(Zacc1)];
X = Amat2*X;

%updating global variables
global state_distX
state_distX = X(1,1);
global state_velX
state_velX = X(2,1);

global state_distY
state_distY = X(4,1);
global state_velY
state_velY = X(5,1);

global state_distZ
state_distZ = X(7,1);
global state_velZ
state_velZ = X(8,1);

global time_prev;
time_prev=time_prev1;

ret_dX=X(1,1);
ret_dY=X(4,1);
ret_dZ=X(7,1);

endfunction


%Serial Capture code implementation
%-------------------------------------------------------------------------------
disp("----------------Settings--------------------------")
if(Serial_enable==1)
  if (exist("serial") == 3) %Check for serial port is supported or not
    disp(" - Serial communication: Supported")
    % Create serial port object and connection settings 
    arduinoState = setSerialSettings(Port, BaudRate)
    
    disp("----------------Data Acquisition------------------")
    countStartNotMatched =1;
    %countStopNotMatched =1;
    while(1)
      %axis ("auto x");
      terminate = 1;
      dataValid=0;
      %---------------------------------------------------------------------------%
      if(terminate==1)
        if(Serial_enable==1)
          startVal= srl_read(arduinoState, 4);
        else
          startVal= [255 255 255 127];%read start value from serial port
        end
        %------------% 
        castValStart=typecast(uint8(startVal(1:4)),"int32");
        castValStart
        if(castValStart==startNum)
          terminate=0;
          disp("start data matched");
        else
          printf(" !!!!!! Start not matched. countStartNotMatched=%d\n", countStartNotMatched);
          countStartNotMatched++;
          
        end;
             %------------%
      end; 
      
      
      if(terminate==0)          
        %disp("getting data");
        if(Serial_enable==1)
          dataValInt32= srl_read(arduinoState, dataValByteCountInt32);%read int32_t data from serial port 
          dataValFloat16= srl_read(arduinoState, dataValByteCountFloat16);%read float16_t data from serial port
          dataValInt16= srl_read(arduinoState, dataValByteCountInt16);%read float16_t data from serial port
        else
          dataVal= [77 66 55 44 33 22 11 9 8 7 6 5 4 3 2 1];
          %[ 77 66 55 44      33 22 11 9     8 7 6 5        4 3 2 1] 
          %  Time(lsb-->msb), Z(lsb-->msb),  Y(lsb-->msb),  X(lsb-->msb)
          %  index 1-4      index 5-8      index 9-12       index 13-16
        end
          %----------%            
        if(Serial_enable==1)
          stopVal= srl_read(arduinoState, 4);
        else
          stopVal= [255 255 255 126];%read stop value from serial port
        end
          %----------%
        CastValStop=typecast(uint8(stopVal(1:4)),"int32");
        if(CastValStop==stopNum)
          dataValid=1;
          recvMsgInt32=dataValInt32;
          for i=1:1:(dataValByteCountInt32/4);
            dataValInt32=recvMsgInt32(dataPosInt32(i)+1:dataPosInt32(i+1));
            recvValueInt32(1,i)=typecast(uint8(dataValInt32),"int32");
          end
          recvMsgFloat16=dataValFloat16;
          for i=1:1:(dataValByteCountFloat16/2);
            dataValFloat16=recvMsgFloat16(dataPosFloat16(i)+1:dataPosFloat16(i+1));
            recvValueFloat16(1,i)=typecast(uint8(dataValFloat16),"int16")/pow2(FIXED_POINT_FRACTIONAL_BITS);
          end
          recvMsgInt16=dataValInt16;
          for i=1:1:(dataValByteCountInt16/2);
            dataValInt16=recvMsgInt16(dataPosInt16(i)+1:dataPosInt16(i+1));
            recvValueInt16(1,i)=typecast(uint8(dataValInt16),"int16");
          end  
          
          %assign ordered X,Y,Z 
          timeval=recvValueInt32(1,1);
          printf(" - timeval=%d\n",timeval);
          posX=recvValueInt32(1,2);
          printf(" - posX=%d\n",posX);
          posY=recvValueInt32(1,3);
          printf(" - posY=%d\n",posY);
          posZ=recvValueInt32(1,4);
          printf(" - posZ=%d\n",posZ);
          Xacc=recvValueFloat16(1,1);
          printf(" - Xacc=%f mm/s2\n",Xacc);
          Yacc=recvValueFloat16(1,2);
          printf(" - Yacc=%f mm/s2\n",Yacc);
          Zacc=recvValueFloat16(1,3);
          printf(" - Zacc=%f mm/s2\n",Zacc);
          Xmag=recvValueFloat16(1,4);
          printf(" - Xmag=%f microTesla\n",Xmag);
          Ymag=recvValueFloat16(1,5);
          printf(" - Ymag=%f microTesla\n",Ymag);
          Zmag=recvValueFloat16(1,6);
          printf(" - Zmag=%f microTesla\n",Zmag);
          Xgyro=recvValueFloat16(1,7);
          printf(" - Xgyro=%f \n",Xgyro);
          Ygyro=recvValueFloat16(1,8);
          printf(" - Ygyro=%f \n",Ygyro);
          Zgyro=recvValueFloat16(1,9);
          printf(" - Zgyro=%f \n",Zgyro);
          Xerr=recvValueInt16(1,1);
          printf(" - Xerr=%d \n",Xerr);
          Yerr=recvValueInt16(1,2);
          printf(" - Yerr=%d \n",Yerr);
          Zerr=recvValueInt16(1,3);
          printf(" - Zerr=%d \n",Zerr);
          XYerr=recvValueInt16(1,4);
          printf(" - XYerr=%d \n",XYerr);
          XZerr=recvValueInt16(1,5);
          printf(" - XZerr=%d \n",XZerr);
          YZerr=recvValueInt16(1,6);
          printf(" - YZerr=%d \n",YZerr);
          
          %Function call kalmanxyz
          [kposX kposY kposZ] = kalmanxyz(timeval,posX,posY,posZ,Xerr,Yerr,Zerr,XYerr,XZerr,YZerr);
          
          %Function call fir_filt
          [f_Xacc] = fir_filt(Xacc);
          %Function call distanceCalc
          [Xdist Ydist Zdist] = distanceCalc(timeval,f_Xacc,Yacc,Zacc);
          disp("-------Distance calculated from ACCELEROMETER ONLY----------\n")
          printf(" - Xdist=%d \n",Xdist);
          printf(" - Ydist=%d \n",Ydist);
          printf(" - Zdist=%d \n",Zdist);
          
          %use below 2d plotting 
          subplot(2,1,1);      
          plot(timeval,posY,'-*');
          hold on;
          subplot(2,1,2);
          %plot(timeval,Xacc,'-o');
          plot(timeval,f_Xacc,'-o');
          hold on;
          
          CastValStop
          disp("stop data matched");
        else    
          dataValid=0;  
        end;
          %----------%
        terminate=1;   
      end;
      disp("-----------------------\n");   
    endwhile
  else
    disp(" - Serial communication: NOT Supported!!!")
  endif
else
  disp("Serial_enable is set to '0' ")
end





