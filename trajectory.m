dt = 0.1;
acce_dur = 1.5;
acce_num = acce_dur/dt;

setpoint = zeros()
vmax = 0.2;%m/s
k = 0.2/1.5;%m/s^2
vlmax = 0.4;
k1 = vlmax/1.5;
for index = 1:98
    if index<=15
        t = index*dt;
        setpoint(index,1) = k*t;
        setpoint(index,2) = 0;
        setpoint(index,3) = 0.5*k*t*t;
        setpoint(index,4) = 0;
    end
    if index<=83&&index>15
        t = index*dt;
        setpoint(index,1) = vmax;
        setpoint(index,2) = 0;
        setpoint(index,3) = 0.5*(t-1.5+t)*vmax;
        setpoint(index,4) = 0;
    end
    if index>83
        t = index*dt;
        setpoint(index,1) = vmax - k*(index-83)*dt;
        setpoint(index,2) = 0;
        setpoint(index,3) = setpoint(83,3)+0.5*(index-83)*dt*(0.2+setpoint(index,1));
        setpoint(index,4) = 0;
    end
end
x1 = setpoint(98,3);y1 = setpoint(98,4);
for index = 99:194
    indexk = index - 98;
    if indexk <=15
        t = indexk*dt;
        setpoint(index,1) = 0;
        setpoint(index,2) = -k1*t;
        setpoint(index,3) = x1;
        setpoint(index,4) = -0.5*k1*t*t+y1;
    end
    if indexk>15 && indexk<=81
         t = indexk*dt;
        setpoint(index,1) = 0;
        setpoint(index,2) = -vlmax;
        setpoint(index,3) = x1;
        setpoint(index,4) = -0.5*vlmax*(t+t-1.5)+y1;
    end
    if indexk>81
        t = indexk*dt;
        setpoint(index,1) = 0;
        setpoint(index,2) = -vlmax + k1*(indexk-81)*dt;
        setpoint(index,3) = x1;
        setpoint(index,4) = setpoint(81+98,4)+0.5*(indexk-81)*dt*(-0.4+setpoint(index,2));
    end
end
x2 = setpoint(194,3);y2 = setpoint(194,4);
for index = 195:292
    indexk = index-194;
    if indexk<=15
        t = indexk*dt;
        setpoint(index,1) = k*t;
        setpoint(index,2) = 0;
        setpoint(index,3) = x2+0.5*k*t*t;
        setpoint(index,4) = y2;
    end
    if indexk>15 && indexk<=83
        t = indexk*dt;
        setpoint(index,1) = vmax;
        setpoint(index,2) = 0;
        setpoint(index,3) = x2+0.5*vmax*(t+t-1.5);
        setpoint(index,4) = y2;
    end
    if indexk>83
        t = indexk*dt;
        setpoint(index,1) = vmax-k*(indexk-83)*dt;
        setpoint(index,2) = 0;
        setpoint(index,3) = setpoint(83+194,3)+0.5*(indexk-83)*dt*(0.2+setpoint(index,1));
        setpoint(index,4) = y2;
    end
end 
x3 = setpoint(292,3);y3 = setpoint(292,4);
% for index = 293:
%     indexk = index-292;
%     if indexk<=15
%         t = indexk*dt;
%         setpoint(index,1) = k1*t;
%         setpoint(index,2) = 0;
%         setpoint(index,3) = x2+0.5*k1*t*t;
%         setpoint(index,4) = y2;
%     end
%     if indexk>15 && indexk<=83
%         t = indexk*dt;
%         setpoint(index,1) = vmax;
%         setpoint(index,2) = 0;
%         setpoint(index,3) = x2+0.5*vmax*(t+t-1.5);
%         setpoint(index,4) = y2;
%     end
%     if indexk>83
%         t = indexk*dt;
%         setpoint(index,1) = vmax-k*(indexk-83)*dt;
%         setpoint(index,2) = 0;
%         setpoint(index,3) = setpoint(83+194,3)+0.5*(indexk-83)*dt*(0.2+setpoint(index,1));
%         setpoint(index,4) = y2;
%     end
% end
