input_x = [0,1.76,1.8516,3.5067,11.414];
input_y = [0,0,-3.2207,-3.2674,2.935];
point_num = length(input_x);
dt = 0.1;
vmax = 0.1;
acce = 0.01;
t_acce = vmax/acce;
index_acce = round(t_acce/dt)+1;
pathlen_acce = 0.5*vmax*t_acce;
pathlen_side = pathlen_acce;
setpoint = cell(1,point_num-1);
sp = [0 0 0 0];

for i = 1:point_num-1
    path = [input_x(i+1) - input_x(i),input_y(i+1) - input_y(i)];
    pathlen = abs(path);
    [maxpalen,xy] = max(pathlen);
    [minpalen,yx] = min(pathlen);
    t_unif = (maxpalen - 2*pathlen_side)/vmax;
    t = t_unif+t_acce*2;
    index_t = round(t/dt)+1;
    index_dece = round((t_unif+t_acce)/dt)+1;
    if (0.5*vmax*t>minpalen)
        small = 1;
        a_min = minpalen*4/t/t;
        index_up_min = round(t/2/dt)+1;
        index_up_max = index_up_min;
    else
        small = 0;
        t_up = minpalen*2/vmax-t;
        t_side = (t - t_up)/2;
        a_min = vmax/t_side;
        index_up_min = round(t_side/dt)+1;
        index_up_max = round((t_up+t_side)/dt)+1;
    end
    for j = 1:index_t
        if (j<=index_acce)
            if(xy==1)
                sp(1) = sign(path(1))*acce*j*dt;
                sp(2) = sign(path(1))*0.5*acce*(j*dt)^2;
                sp(3) = sign(path(2))*a_min*j*dt;
                sp(4) = sign(path(2))*0.5*a_min*(j*dt)^2;
            else
                sp(1) = sign(path(1))*a_min*j*dt;
                sp(2) = sign(path(1))*0.5*a_min*(j*dt)^2;
                sp(3) = sign(path(2))*acce*j*dt;
                sp(4) = sign(path(2))*0.5*acce*(j*dt)^2;
            end
        end
        if (j>index_acce && j<=index_up_min)
            if(xy==1)
                sp(1) = sign(path(1))*vmax;
                sp(2) = sign(path(1))*vmax*(j-index_acce)*dt+setpoint{i}(index_acce,2);
                sp(3) = sign(path(2))*a_min*j*dt;
                sp(4) = sign(path(2))*0.5*a_min*(j*dt)^2;
            else
                sp(1) = sign(path(1))*a_min*j*dt;
                sp(2) = sign(path(1))*0.5*a_min*(j*dt)^2;
                sp(3) = sign(path(2))*vmax;
                sp(4) = sign(path(2))*vmax*(j-index_acce)*dt+setpoint{i}(index_acce,4);
            end
        end
        if(j>index_up_min && j<=index_up_max)
            if(xy==1)
                sp(1) = sign(path(1))*vmax;
                sp(2) = sign(path(1))*vmax*(j-index_acce)*dt+setpoint{i}(index_acce,2);
                sp(3) = sign(path(2))*vmax;
                sp(4) = sign(path(2))*vmax*(j-index_up_min)*dt+setpoint{i}(index_up_min,4);
            else
                sp(1) = sign(path(1))*vmax;
                sp(2) = sign(path(1))*vmax*(j-index_up_min)*dt+setpoint{i}(index_up_min,2);
                sp(3) = sign(path(2))*vmax;
                sp(4) = sign(path(2))*vmax*(j-index_acce)*dt+setpoint{i}(index_acce,4);
            end
        end
        if(j>index_up_max && j<=index_dece)
            if(xy==1)
                sp(1) = sign(path(1))*vmax;
                sp(2) = sign(path(1))*vmax*(j-index_acce)*dt+setpoint{i}(index_acce,2);
                if(small==1)
                    sp(3) = sign(path(2))*(abs(setpoint{i}(index_up_max,3))-a_min*(j-index_up_max)*dt);
                    sp(4) = sign(path(2))*0.5*(abs(sp(3))+abs(setpoint{i}(index_up_max,3)))*(j-index_up_max)*dt+setpoint{i}(index_up_max,4);
                else
                    sp(3) = sign(path(2))*(vmax-a_min*(j-index_up_max)*dt);
                    sp(4) = sign(path(2))*0.5*(abs(sp(3))+vmax)*(j-index_up_max)*dt+setpoint{i}(index_up_max,4);
                end
            else
                if(small==1)
                    sp(1) = sign(path(1))*(abs(setpoint{i}(index_up_max,1))-a_min*(j-index_up_max)*dt);
                    sp(2) = sign(path(1))*0.5*(abs(sp(1))+abs(setpoint{i}(index_up_max,1)))*(j-index_up_max)*dt+setpoint{i}(index_up_max,2);
                else
                    sp(1) = sign(path(1))*(vmax-a_min*(j-index_up_max)*dt);
                    sp(2) = sign(path(1))*0.5*(abs(sp(1))+vmax)*(j-index_up_max)*dt+setpoint{i}(index_up_max,2);
                end
                sp(3) = sign(path(2))*vmax;
                sp(4) = sign(path(2))*vmax*(j-index_acce)*dt+setpoint{i}(index_acce,4);
            end
        end
        if(j>index_dece)
            if(xy==1)
                sp(1) = sign(path(1))*(vmax-acce*(j-index_dece)*dt);
                sp(2) = sign(path(1))*0.5*(vmax+abs(sp(1)))*(j-index_dece)*dt+setpoint{i}(index_dece,2);
                if(small==1)
                    sp(3) = sign(path(2))*(abs(setpoint{i}(index_up_max,3))-a_min*(j-index_up_max)*dt);
                    sp(4) = sign(path(2))*0.5*(abs(sp(3))+abs(setpoint{i}(index_up_max,3)))*(j-index_up_max)*dt+setpoint{i}(index_up_max,4);
                else
                    sp(3) = sign(path(2))*(vmax-a_min*(j-index_up_max)*dt);
                    sp(4) = sign(path(2))*0.5*(abs(sp(3))+vmax)*(j-index_up_max)*dt+setpoint{i}(index_up_max,4);
                end
                
            else
               if(small==1)
                    sp(1) = sign(path(1))*(abs(setpoint{i}(index_up_max,1))-a_min*(j-index_up_max)*dt);
                    sp(2) = sign(path(1))*0.5*(abs(sp(1))+abs(setpoint{i}(index_up_max,1)))*(j-index_up_max)*dt+setpoint{i}(index_up_max,2);
                else
                    sp(1) = sign(path(1))*(vmax-a_min*(j-index_up_max)*dt);
                    sp(2) = sign(path(1))*0.5*(abs(sp(1))+vmax)*(j-index_up_max)*dt+setpoint{i}(index_up_max,2);
                end
                sp(3) = sign(path(2))*(vmax-acce*(j-index_dece)*dt);
                sp(4) = sign(path(2))*0.5*(vmax+abs(sp(3)))*(j-index_dece)*dt+setpoint{i}(index_dece,4);
            end
        end
        setpoint{i} = [setpoint{i};sp];
    end
end
velocity_x = [];
velocity_y = [];
position_x = [];
position_y = [];
for i = 1:point_num-1
    velocity_x = [velocity_x;setpoint{i}(:,1)];
    velocity_y = [velocity_y;setpoint{i}(:,3)];
    if i>=2
        setpoint{i}(:,2) = setpoint{i}(:,2) + setpoint{i-1}(end,2);
        setpoint{i}(:,4) = setpoint{i}(:,4) + setpoint{i-1}(end,4);
    end
    position_x = [position_x;setpoint{i}(:,2)];
    position_y = [position_y;setpoint{i}(:,4)];
end
trajectory = [velocity_x position_x velocity_y position_y];