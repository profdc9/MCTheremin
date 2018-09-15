% octave script
function y = genwaves(n)

y=floor((0.5+0.5*cos((2*pi*(0:n-1))/n))*(n-1));
fp=fopen('waves.cpp','w');
fprintf(fp,'const unsigned short sinewave[%d] = {',n);
fprintf(fp,'%g,',y(1:length(y)-1));
fprintf(fp,'%g',y(length(y)));
fprintf(fp,' };\r\n');

y=zeros(1,n);
y(1:n/2)=(0:n/2-1)*2;
y(n/2+1:n)=(n/2-1:-1:0)*2+1;
fprintf(fp,'const unsigned short triangle[%d] = {',n);
fprintf(fp,'%g,',y(1:length(y)-1));
fprintf(fp,'%g',y(length(y)));
fprintf(fp,' };\r\n');

y=floor((0.5+0.75*cos((2*pi*(0:n-1))/n))*(n-1));
y=max(min(y,n-1),0);
fprintf(fp,'const unsigned short distorted[%d] = {',n);
fprintf(fp,'%g,',y(1:length(y)-1));
fprintf(fp,'%g',y(length(y)));
fprintf(fp,' };\r\n');

y=floor((0.5+0.5*cos((2*pi*(0:n-1))/n).^3)*(n-1));
y=max(min(y,n-1),0);
fprintf(fp,'const unsigned short distorted2[%d] = {',n);
fprintf(fp,'%g,',y(1:length(y)-1));
fprintf(fp,'%g',y(length(y)));
fprintf(fp,' };\r\n');

fprintf(fp,'const unsigned short *wavelist[4] = {sinewave,triangle,distorted,distorted2};\r\n');
y=0;
fclose(fp);
