function [vx_f1,vy_f1,vx_f2,vy_f2] = collide_particles(m1,x1,y1,vx1,vy1,m2,x2,y2,vx2,vy2)
%collide_particles Function computes collided particles velocity
%   m1, m2 - mass of particles
%   x1,y1, x2,y2 - coordinates of particles
%   vx1,vy1, vx2,vy2 - velocity components 

theta=atan2(y2-y1,x2-x1);

VN1=(vx1*cos(theta))+(vy1*sin(theta));
VT1=(-vx1*sin(theta))+(vy1*cos(theta));
VN2=(vx2*cos(theta))+(vy2*sin(theta));
VT2=(-vx2*sin(theta))+(vy2*cos(theta));

VN1_F=((VN1*(m1-m2))+(2*m2*VN2))/(m1+m2);
VN2_F=((VN2*(m2-m1))+(2*m1*VN1))/(m1+m2);

vx_f1=(VN1_F*cos(theta))-(VT1*sin(theta));
vy_f1=(VN1_F*sin(theta))+(VT1*cos(theta));
vx_f2=(VN2_F*cos(theta))-(VT2*sin(theta));
vy_f2=(VN2_F*sin(theta))+(VT2*cos(theta));

end

