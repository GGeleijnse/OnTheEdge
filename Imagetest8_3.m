%%
% Copyright (c) 2021, Erasmus University Medical Center Rotterdam 
% ("Erasmus MC"). All rights reserved. 
% 
% Non-commercial use of the MATLAB program for research purposes is 
% permitted provided that the following conditions are met:
% 
% *	The use of the MATLAB program must be duly acknowledged and all 
% advertising materials mentioning features or use of the MATLAB program
% must display the following acknowledgement: 
% Erasmus MC. G. Geleijnse et al 2021. "Measuring Image Quality of ENT 
% Chip-on-tip Endoscopes, Journal of Imaging Science and Technology, 
% vol. 65, no. 2, 2021.?.
% *	Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
% *	Any change made to the original MATLAB program needs to be clearly 
% indicated in the header of the source-code and accompanied documents; 
% *	Changes and improvements need to be shared with the Erasmus MC, 
% att. Geert Geleijnse; g.geleijnse@erasmusmc.nl;
% *	It is understood and agreed that any change or improvement made to the
% original MATLAB program will be owned by Erasmus MC as being part of the
% original work, unless the improvement can run independently from (and
% without the use of) the original program and needs to be considered as a 
% new 'work' according to the Dutch Copyright Act (Dutch: "Auteurswet");
% *	Neither the name of Erasmus MC nor the names of its contributors may be
% used to endorse or promote products derived from this software without
% specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY ERASMUS MC 'AS IS' AND ANY EXPRESS OR 
% IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
% OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
% IN NO EVENT SHALL ERASMUS MC BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
% NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
% DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
% THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
% THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
% 
% For the avoidance of doubt, commercial use of any kind (including but not
% limited to direct and indirect exploitation, validation of commercial
% products etc.) is prohibited without approval of Erasmus MC and requires 
% a commercial license to be negotiated by Erasmus MC?s Technology Transfer 
% Office; tto@erasmusmc.nl.
% 
% If you are contacted by any party interested in the use of the MATLAB
% program, please refer them to Erasmus MC, Department of Ear, Nose, Throat
% (ENT), Geert Geleijnse: g.geleijnse@erasmusmc.nl.


%%


clc
clear all
close all

fname = '141007_05' 
%
Image = imread([fname,'.bmp'],'bmp');
Image = Image(:,50:1200,:);
Image = imrotate(Image,180);

% Gray reference values provided by Image science associates for the Rez
% checker target matte.
GrayRGBrefs=[242,241,234;222,223,220;207,208,204;187,189,186;...
             169,170,168;148,149,148;133,135,133;114,116,115;...
             96,97,96;73,74,74;58,57,58;54,53,54];
TdensMatt = [0.06, 0.13 0.2 0.3 0.4 0.52 0.62 0.76 0.93 1.17 1.38 1.44];

%% Detect the black circles on the Reze Checker Target Matte
% set the radii in units of pixels for detecting the black circles on the
% Rez checker target matte
Rmin = 15;
Rmax = 25;

% Find the black circles on the Rez checker target for reference.
[centers, radii] = imfindcircles(Image, [Rmin Rmax], ...
    'ObjectPolarity','dark','Method','TwoStage','Sensitivity',0.75,'EdgeThreshold',0.10);
% Determine in which quadrant the detected circles are.
[Ymax Xmax Zmax] = size(Image);
Ill = find(centers(:,2) > Ymax/2 & centers(:,1) < Xmax/2,1,'first');
Cll = centers(Ill,:)
Ilr = find(centers(:,2) > Ymax/2 & centers(:,1) > Xmax/2,1,'first');
Clr = centers(Ilr,:)
Iur = find(centers(:,2) < Ymax/2 & centers(:,1) > Xmax/2,1,'first');
Cur = centers(Iur,:)

% Horizontal and vertical resolution in pixels per mm.
ResX = (Clr(1) - Cll(1))/16 % pixels per mm
ResY = (Clr(2) - Cur(2))/19 % pixels per mm

% Plot the image with the detected circles.
figure(1) 
set(gcf,'Color','w')
subplot(231)
imshow(Image);
viscircles(centers, radii,'EdgeColor','y');
viscircles(Cll, radii(Ill),'EdgeColor','g');
text(Cll(1)-radii(Ill)*3,Cll(2),['\color{green}LL'])
viscircles(Clr, radii(Ilr),'EdgeColor','r');
text(Clr(1)+radii(Ilr)*1.5,Clr(2),['\color{red}LR'])
viscircles(Cur, radii(Iur),'EdgeColor','b');
text(Cur(1)+radii(Iur)*1.5,Cur(2),['\color{blue}UR'])
title('Circle detection')
%% Determine the Regions of Interest

figure(1) 
subplot(231)

% Width (w) and height (h) for the ROI's on the slanted edges
w = (Clr(1)-Cll(1))/5;
h = (Clr(2)-Cur(2))/6;
w = (w + h)/2;
h = w;
% colored patches (p1) should be smaller than the stepresponses (p2)
p1 = 0.7;
p2 = 0.4;
shiftx=0.*w;
shifty=-0.*h;
% Determine the center coordinates of the target based on the 
% detected circles on the target and mark it with a cross.
Cxmid = (Cll(1) + (Clr(1)+Cur(1))/2)/2 + shiftx;
Cymid = (Cur(2) + (Clr(2)+Cll(2))/2)/2 + shifty;
line([Cxmid-10,Cxmid+10],[Cymid,Cymid],'LineWidth',2,'Color',[1 0 0])
line([Cxmid,Cxmid],[Cymid-10,Cymid+10],'LineWidth',2,'Color',[1 0 0])
Cx  = floor(Cxmid + ([-3:1:2]+p1/2).*w);
Cy  = floor(Cymid + ([-3.5:1:2.5]+p1/2).*h);
%Cx  = floor((Cur(1)+Clr(1))/2+ ([-5.5:1:-0.5]+p/2).*w +shiftx*w);
%Cy  = floor((Cll(2)+Clr(2))/2+ ([-6.5:1:-0.5]+p/2).*h +shifty*h);

%% This section is not ready for correcting radial barrel distortion
[xi,yi] = meshgrid(Cx,Cy);

imid = round(size(Image)/2); % Find index of middle element
line([imid(2)-10,imid(2)+10],[imid(1),imid(1)],'LineWidth',2,'Color',[0 1 0])
line([imid(2),imid(2)],[imid(1)-10,imid(1)+10],'LineWidth',2,'Color',[0 1 0])
xt = xi - imid(2);
yt = yi - imid(1);
[theta,r] = cart2pol(xt,yt);
a = -.00000035; % Try varying the amplitude of the cubic term.
s = r + a*r.^3;
[ut,vt] = pol2cart(theta,s);

Cxt = ut + imid(2);
Cyt = vt + imid(1);
%% Continue with determining the Regions of Interest

Wp1  = floor((1-p1)*w);
Hp1  = floor((1-p1)*h);
Wp2  = floor((1-p2)*w);
Hp2  = floor((1-p2)*h); 
%
Y_1 = Clr(2)+(-13+p1)/2*h;

% Colored patches on the top row 
n=1;% line 1
rectangle('Position', [Cx(2),Cy(n),Wp1,Hp1],'EdgeColor','y')

rectangle('Position', [Cx(3),Cy(n),Wp1,Hp1],'EdgeColor','y')

rectangle('Position', [Cx(4),Cy(n),Wp1,Hp1],'EdgeColor','y')

rectangle('Position', [Cx(5),Cy(n),Wp1,Hp1],'EdgeColor','y')

n=2;% line 2
rectangle('Position', [Cx(1),Cy(n),Wp1,Hp1],'EdgeColor','y')
% you can fine tune the horizontal and vertical coordinates of the vertical
% slanted edge in the next two lines of code.
Cx_22 = floor(Cxmid + (-2+p2/2).*w+10); 
Cy_22 = floor(Cymid + (-2.5+p2/2).*h-5);
Hp2_22 = floor((2-p2)*h-5);
rectangle('Position', [Cx_22,Cy_22,Wp2,Hp2_22],'EdgeColor','y')
rectangle('Position', [floor(Cxmid + (-1+p2/2).*w),floor(Cymid + (-2.5+p2/2).*h),(2-p2)*w,Hp2],'EdgeColor','y')
rectangle('Position', [floor(Cxmid + (1+p2/2).*w),floor(Cymid + (-2.5+p2/2).*h),Wp2,(2-p2)*h],'EdgeColor','y')
rectangle('Position', [Cx(6),Cy(n),Wp1,Hp1],'EdgeColor','y')
n=3;% line 3
rectangle('Position', [Cx(1),Cy(n),Wp1,Hp1],'EdgeColor','y')
% you can fine tune the horizontal and vertical coordinates of the 
% horizontal slanted edge in the next two lines of code.
Cx_33 = floor(Cxmid + (-1+p2/2).*w+0);
Cy_33 = floor(Cymid + (-1.5+p2/2).*h-5-20);
Wp2_33 = floor((2-p2)*w);
rectangle('Position', [Cx_33,Cy_33,Wp2_33,Hp2],'EdgeColor','y')
rectangle('Position', [Cx(6),Cy(n),Wp1,Hp1],'EdgeColor','y')
n=4;% line 4
rectangle('Position', [Cx(1),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(2),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(3),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(4),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(5),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(6),Cy(n),Wp1,Hp1],'EdgeColor','y')

n=5;% line 5
rectangle('Position', [Cx(1),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(2),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(3),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(4),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(5),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(6),Cy(n),Wp1,Hp1],'EdgeColor','y')

n=6;% line 6
rectangle('Position', [Cx(1),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(2),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(3),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(4),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(5),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(6),Cy(n),Wp1,Hp1],'EdgeColor','y')

n=7;% line 7
rectangle('Position', [Cx(2),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(3),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(4),Cy(n),Wp1,Hp1],'EdgeColor','y')
rectangle('Position', [Cx(5),Cy(n),Wp1,Hp1],'EdgeColor','y')

title('Applied grid')


%% Analyze the gray patches
% line 4
Im24 = Image(Cy(4):Cy(4)+Hp1,Cx(2):Cx(2)+Wp1,:);
[Im24_avg,Im24_std,Im24_RGBavg] = RGBlum(Im24);
[n,m,c] = size(Im24);
Npix  = n*m;

Im34 = Image(Cy(4):Cy(4)+Hp1,Cx(3):Cx(3)+Wp1,:);
[Im34_avg,Im34_std,Im34_RGBavg] = RGBlum(Im34);
Im44 = Image(Cy(4):Cy(4)+Hp1,Cx(4):Cx(4)+Wp1,:);
[Im44_avg,Im44_std,Im44_RGBavg] = RGBlum(Im44);
Im54 = Image(Cy(4):Cy(4)+Hp1,Cx(5):Cx(5)+Wp1,:);
[Im54_avg,Im54_std,Im54_RGBavg] = RGBlum(Im54);

% line 5
Im25 = Image(Cy(5):Cy(5)+Hp1,Cx(2):Cx(2)+Wp1,:);
[Im25_avg,Im25_std,Im25_RGBavg] = RGBlum(Im25);
Im35 = Image(Cy(5):Cy(5)+Hp1,Cx(3):Cx(3)+Wp1,:);
[Im35_avg,Im35_std,Im35_RGBavg] = RGBlum(Im35);
Im45 = Image(Cy(5):Cy(5)+Hp1,Cx(4):Cx(4)+Wp1,:);
[Im45_avg,Im45_std,Im45_RGBavg] = RGBlum(Im45);
Im55 = Image(Cy(5):Cy(5)+Hp1,Cx(5):Cx(5)+Wp1,:);
[Im55_avg,Im55_std,Im55_RGBavg] = RGBlum(Im55);

% line 6
Im26 = Image(Cy(6):Cy(6)+Hp1,Cx(2):Cx(2)+Wp1,:);
[Im26_avg,Im26_std,Im26_RGBavg] = RGBlum(Im26);
Im36 = Image(Cy(6):Cy(6)+Hp1,Cx(3):Cx(3)+Wp1,:);
[Im36_avg,Im36_std,Im36_RGBavg] = RGBlum(Im36);
Im46 = Image(Cy(6):Cy(6)+Hp1,Cx(4):Cx(4)+Wp1,:);
[Im46_avg,Im46_std,Im46_RGBavg] = RGBlum(Im46);
Im56 = Image(Cy(6):Cy(6)+Hp1,Cx(5):Cx(5)+Wp1,:);
[Im56_avg,Im56_std,Im56_RGBavg] = RGBlum(Im56);

% Plot all the luminance values per gray patch
subplot(234)
plot(...
    [reshape(rgb2gray(Im24),1,[]),reshape(rgb2gray(Im34),1,[]),reshape(rgb2gray(Im44),1,[]),reshape(rgb2gray(Im54),1,[])...
     reshape(rgb2gray(Im25),1,[]),reshape(rgb2gray(Im35),1,[]),reshape(rgb2gray(Im45),1,[]),reshape(rgb2gray(Im55),1,[])...
     reshape(rgb2gray(Im26),1,[]),reshape(rgb2gray(Im36),1,[]),reshape(rgb2gray(Im46),1,[]),reshape(rgb2gray(Im56),1,[])]...
    ,'.','Color',[0.6 0.6 0.6],'MarkerSize',1)
hold on
i=0;
for m =4:6
    for n = 2:5
        eval(['plot([ (', num2str(i), '+1)*Npix, (', num2str(i),...
              '+1)*Npix],[0,300], ''k'', ''LineWidth'',1 )'] )';
        eval(['plot([ (', num2str(i), '+1/8)*Npix, (', num2str(i),...
              '+7/8)*Npix],[Im', num2str(n), num2str(m), '_avg, Im',...
              num2str(n), num2str(m), '_avg], ''r'', ''LineWidth'',2 )'] )';
        eval(['plot([ (', num2str(i), '+1/4)*Npix, (', num2str(i),...
              '+3/4)*Npix],[Im', num2str(n), num2str(m), '_avg + Im', ...
              num2str(n), num2str(m),'_std, Im', num2str(n), num2str(m),...
              '_avg + Im', num2str(n), num2str(m), '_std], ''r'', ''LineWidth'',1 )'] )';
        eval(['plot([ (', num2str(i), '+1/4)*Npix, (', num2str(i),...
              '+3/4)*Npix],[Im', num2str(n), num2str(m), '_avg - Im', ...
              num2str(n), num2str(m),'_std, Im', num2str(n), num2str(m),...
              '_avg - Im', num2str(n), num2str(m), '_std], ''r'', ''LineWidth'',1 )'] )';
         eval(['plot([ (', num2str(i), '+1/2)*Npix, (', num2str(i),...
              '+1/2)*Npix],[Im', num2str(n), num2str(m), '_avg - Im', ...
              num2str(n), num2str(m),'_std, Im', num2str(n), num2str(m),...
              '_avg + Im', num2str(n), num2str(m), '_std], ''r'', ''LineWidth'',1 )'] )';
         eval(['rectangle(''Position'', [(',num2str(i),')*Npix,-40,Npix,40]',...
              ', ''FaceColor'',GrayRGBrefs(',num2str(i+1),',:)/255, ''EdgeColor'' ,''k'')'])
         i=i+1;
    end
end
axis([0 12*Npix -40 300])
xlabel('Gray patch')
set(gca,'xticklabel',{[]})
ylabel('Luminance channel Y [-]')
title('Luminance values per gray patch')

% Summary values of the gray patches.
GrayAvgs = [Im24_avg,Im34_avg,Im44_avg,Im54_avg...
      Im25_avg,Im35_avg,Im45_avg,Im55_avg...
      Im26_avg,Im36_avg,Im46_avg,Im56_avg];
GrayStds = [Im24_std,Im34_std,Im44_std,Im54_std...
      Im25_std,Im35_std,Im45_std,Im55_std...
      Im26_std,Im36_std,Im46_std,Im56_std];
RGBAvgs  = [Im24_RGBavg;Im34_RGBavg;Im44_RGBavg;Im54_RGBavg;...
      Im25_RGBavg;Im35_RGBavg;Im45_RGBavg;Im55_RGBavg;...
      Im26_RGBavg;Im36_RGBavg;Im46_RGBavg;Im56_RGBavg];


% OECF first order fit based on pixel levels between minimum plus 10% range
% and maximum minus 10% range. 
GrayMax = max(GrayAvgs);
GrayMin = min(GrayAvgs);
GrayRng = GrayMax-GrayMin;
idcs = find(( GrayAvgs> GrayMin+0.1*GrayRng) & (GrayAvgs < GrayMax-0.1*GrayRng));
P = polyfit(TdensMatt(idcs), log10(GrayAvgs(idcs)/255),1);
gamma = -1*P(1);   

% plot the data and linear fit.
subplot(232)
plot(TdensMatt,log10(GrayAvgs/255),'.-k', 'LineWidth',1)
hold on
plot(TdensMatt,log10(RGBAvgs(:,1)/255),'.-', 'LineWidth',1,'Color',[0.8 0.1 0.1])
plot(TdensMatt,log10(RGBAvgs(:,2)/255),'.-', 'LineWidth',1,'Color',[0.1 0.6 0.1])
plot(TdensMatt,log10(RGBAvgs(:,3)/255),'.-', 'LineWidth',1,'Color',[0.1 0.1 0.8])
plot(TdensMatt,polyval(P,TdensMatt),'--k');
grid on
xlabel('Status T density')
ylabel('log(pixel level / 255)')
legend('Lum','Red','Green','Blue','Lum 1st-order fit','Location','NorthEast')
text( 0.1,polyval(P,TdensMatt(end-1)),['gamma = ', num2str(-1*P(1))])

%% Analyze the colored patches
% convert the image to xyz color space.
Image_xyz = rgb2xyz(double(Image)./255);

load RefColors

figure(1)
subplot(235)
plotChromaticity
title('Chromaticity diagram')
hold on

i=0;
% Next lines were used to test the 'eval' commands.
%Im64_C = squeeze(mean(mean(double(Image(Cy(6):Cy(6)+Hp1,Cx(4):Cx(4)+Wp,:)),2),1))'
%plot(Im64_x,Im64_y,'Color',Im64_C)

% Obtain the data in each colored ROI and plot the data in the chromaticity
% diagram with error ellipses.

for m = [2] %column
    for n = [4] % row
    eval(['Im', num2str(n), num2str(m),'_S = sum(double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,:)),3);']);
    eval(['Im', num2str(n), num2str(m),'_C = transpose(squeeze(mean(mean(double(Image(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,:)),2),1)./255));']);
 %Im64_C = squeeze(mean(mean(double(Image_xyz(Cy(6):Cy(6)+Hp1,Cx(4):Cx(4)+Wp1,:)),2),1));
    eval(['Im', num2str(n), num2str(m),'_x = double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,1))./Im',num2str(n), num2str(m),'_S;']);
    eval(['Im', num2str(n), num2str(m),'_y = double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,2))./Im',num2str(n), num2str(m),'_S;']);
    eval(['plot(Im', num2str(n), num2str(m),'_x,Im', ...
          num2str(n), num2str(m),'_y,''.'',''Color'',Im',num2str(n), num2str(m),'_C)']);
    end
end


for m = [2,3,4,5 ] %column
    for n = [1,7] % row
    eval(['Im', num2str(n), num2str(m),'_S = sum(double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,:)),3);']);
    eval(['Im', num2str(n), num2str(m),'_C = transpose(squeeze(mean(mean(double(Image(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,:)),2),1)./255));']);
 %Im64_C = squeeze(mean(mean(double(Image_xyz(Cy(6):Cy(6)+Hp1,Cx(4):Cx(4)+Wp1,:)),2),1));
    eval(['Im', num2str(n), num2str(m),'_x = double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,1))./Im',num2str(n), num2str(m),'_S;']);
    eval(['Im', num2str(n), num2str(m),'_y = double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,2))./Im',num2str(n), num2str(m),'_S;']);
    eval(['plot(Im', num2str(n), num2str(m),'_x,Im', ...
          num2str(n), num2str(m),'_y,''.'',''Color'',Im',num2str(n), num2str(m),'_C)']);
    end
end

for m = [1,6 ] %column
    for n = [2,3,4,5,6] % row
    eval(['Im', num2str(n), num2str(m),'_S = sum(double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,:)),3);']);
    eval(['Im', num2str(n), num2str(m),'_C = transpose(squeeze(mean(mean(double(Image(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,:)),2),1)./255));']);
 %Im64_C = squeeze(mean(mean(double(Image_xyz(Cy(6):Cy(6)+Hp1,Cx(4):Cx(4)+Wp1,:)),2),1));
    eval(['Im', num2str(n), num2str(m),'_x = double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,1))./Im',num2str(n), num2str(m),'_S;']);
    eval(['Im', num2str(n), num2str(m),'_y = double(Image_xyz(Cy(',...
          num2str(n),'):Cy(',num2str(n),')+Hp1,Cx(',num2str(m),'):Cx(',...
          num2str(m),')+Wp1,2))./Im',num2str(n), num2str(m),'_S;']);
    eval(['plot(Im', num2str(n), num2str(m),'_x,Im', ...
          num2str(n), num2str(m),'_y,''.'',''Color'',Im',num2str(n), num2str(m),'_C)']);
    end
end

for m = [2 ] %column
    for n = [4] % row
        eval(['[avg_x', num2str(n), num2str(m), ',avg_y',num2str(n),...
        num2str(m),',r_ellipse',num2str(n), num2str(m),'] = ErrEllipse(Im'...
        num2str(n), num2str(m),'_x,Im',num2str(n), num2str(m),'_y);']);
        eval(['plot(r_ellipse',num2str(n), num2str(m),'(:,1) + avg_x',...
        num2str(n), num2str(m),',r_ellipse',num2str(n), num2str(m),'(:,2) + avg_y',num2str(n), num2str(m), ',''-r'')'])
        eval(['plot(avg_x',num2str(n), num2str(m),',avg_y',num2str(n), num2str(m),',''+r'')'])
        
        eval(['dummy = rgb2xyz(Im',num2str(n), num2str(m),'_Cref/255);'])
        dummy_x = dummy(1)./sum(dummy);
        dummy_y = dummy(2)./sum(dummy);
        plot(dummy_x,dummy_y,'or')
        eval(['plot([dummy_x, avg_x',num2str(n), num2str(m),'],[dummy_y, avg_y',num2str(n), num2str(m),'],''r'')'])
    end
end

for m = [1,6 ] %column
    for n = [2,3,4,5,6] % row
        eval(['[avg_x', num2str(n), num2str(m), ',avg_y',num2str(n),...
        num2str(m),',r_ellipse',num2str(n), num2str(m),'] = ErrEllipse(Im'...
        num2str(n), num2str(m),'_x,Im',num2str(n), num2str(m),'_y);']);
        eval(['plot(r_ellipse',num2str(n), num2str(m),'(:,1) + avg_x',...
        num2str(n), num2str(m),',r_ellipse',num2str(n), num2str(m),'(:,2) + avg_y',num2str(n), num2str(m), ',''-r'')'])
        eval(['plot(avg_x',num2str(n), num2str(m),',avg_y',num2str(n), num2str(m),',''+r'')'])
        
        eval(['dummy = rgb2xyz(Im',num2str(n), num2str(m),'_Cref/255);'])
        dummy_x = dummy(1)./sum(dummy);
        dummy_y = dummy(2)./sum(dummy);
        plot(dummy_x,dummy_y,'or')
        eval(['plot([dummy_x, avg_x',num2str(n), num2str(m),'],[dummy_y, avg_y',num2str(n), num2str(m),'],''r'')'])
    end
end

for m = [2,3,4,5 ] %column
    for n = [1,7] % row
        eval(['[avg_x', num2str(n), num2str(m), ',avg_y',num2str(n),...
        num2str(m),',r_ellipse',num2str(n), num2str(m),'] = ErrEllipse(Im'...
        num2str(n), num2str(m),'_x,Im',num2str(n), num2str(m),'_y);']);
        eval(['plot(r_ellipse',num2str(n), num2str(m),'(:,1) + avg_x',...
        num2str(n), num2str(m),',r_ellipse',num2str(n), num2str(m),'(:,2) + avg_y',num2str(n), num2str(m), ',''-k'')'])
        eval(['plot(avg_x',num2str(n), num2str(m),',avg_y',num2str(n), num2str(m),',''+k'')'])
        
        eval(['dummy = rgb2xyz(Im',num2str(n), num2str(m),'_Cref/255);'])
        dummy_x = dummy(1)./sum(dummy);
        dummy_y = dummy(2)./sum(dummy);
        plot(dummy_x,dummy_y,'ok')
        eval(['plot([dummy_x, avg_x',num2str(n), num2str(m),'],[dummy_y, avg_y',num2str(n), num2str(m),'],''k'')'])
    end
end
%% Calculate color errors

figure(1)
subplot(236)
hold on

i=1;
for m = [2 ] %column
    for n = [4] % row
        eval(['dE', num2str(n), num2str(m), ' =  dE2000(Im',num2str(n), num2str(m) ,'_C,Im',num2str(n), num2str(m),'_Cref./255);']);
        eval(['plot(i,dE',num2str(n), num2str(m),...
            ',''s'',''Color'',abs(Im',num2str(n), num2str(m),'_Cref)./255,',...
            '''MarkerFaceColor'',''w'',''MarkerEdgeColor'',''k'',''MarkerSize'',7,''LineWidth'',2)'])     
        i=i+1;
    end
end

m=1
n=6

for m = [1,6 ] %column
    for n = [2,3,4,5,6] % row
        eval(['dE', num2str(n), num2str(m), ' =  dE2000(Im',num2str(n), num2str(m) ,'_C,Im',num2str(n), num2str(m),'_Cref./255);']);
        eval(['plot(i,dE',num2str(n), num2str(m),...
            ',''s'',''Color'',abs(Im',num2str(n), num2str(m),'_Cref)./255,',...
            '''MarkerFaceColor'',abs(Im',num2str(n), num2str(m),'_Cref)./255)'])     
        i=i+1;
    end
end


for m = [2,3,4,5 ] %column
    for n = [1,7] % row
        eval(['dE', num2str(n), num2str(m), ' =  dE2000(Im',num2str(n), num2str(m) ,'_C,Im',num2str(n), num2str(m),'_Cref./255);']);
        eval(['plot(i,dE',num2str(n), num2str(m),...
            ',''s'',''Color'',abs(Im',num2str(n), num2str(m),'_Cref)./255,',...
            '''MarkerFaceColor'',abs(Im',num2str(n), num2str(m),'_Cref)./255)'])     
        i=i+1;        
    end
end

axis([0, i, 0 30])
xlabel('index [-]')
ylabel('CIE 00 \DeltaE [-]')
title('Color difference')
grid on

% Summary color differences (CIE dE 2000)
Cdiffs = [dE42; dE21; dE31; dE41; dE51; dE61; dE26; dE36; dE46; dE56; dE66;...
          dE12; dE13; dE14; dE15; dE72; dE73; dE74; dE75];

%% Analyze the vertical slanted edge (horizontal step response)
% ROI slanted edge
Im22  = Image(Cy_22:Cy_22+Hp2_22,Cx_22:Cx_22+Wp2,:);

%convert the RGB-integers to gray double
dummy    = double((Im22));
Im22gray = 0.2126*dummy(:,:,1) + 0.7152*dummy(:,:,2) + 0.0722*dummy(:,:,3);
% linearize the gray values
Im22gray = (Im22gray/255).^(1/gamma)*255;

clear dummy
% obtain the number of pixels in x- and y-direction
[n,m] = size(Im22gray);
% filter image to get rid of fiber artefacts
[b,a] = butter(2,1/8,'low');
Im22gray_filt = filtfilt(b,a,Im22gray');
Im22gray_filt = Im22gray_filt';
% calculate the impuls response by diferentiating the step response
Im22diff = diff(Im22gray_filt,1,2);
% estimate x-coordinates of the edge based on the maximum values of the 
% differentiated step respons
[M Im22edge] = max(Im22diff,[],2);
% improve the estimation by fitting a straight line through the estimates
Im22_Y = [1:1:n]';

P = polyfit(Im22_Y, Im22edge+0.5,1);
Im22edgfit = polyval(P,Im22_Y);
% use meshgrid as a quick way to get a matrix containg all x-values.
[X,Y] = meshgrid([1:1:m],[1:1:n]);
% allign all impulse responses
X_edge = X-Im22edgfit;
X_edge_horzchk = X-Im22edgfit;
% change the format to resample the impulse response
X_edge = X_edge(:);
Y_edge = Im22gray(:);
data_horz = [X_edge ,Y_edge];
data_horz = sortrows(data_horz,1);
% calculate the angle of the slanted edge
alpha_horz = atan(P(1));
% transform the x-coordinates to the tilted x-coordinates
data_horz(:,1) = data_horz(:,1)./cos(alpha_horz);

% resample the impulse response to get a constant sample frequency
Fs   = 4; % samples per pixel
bins = (floor(data_horz(1,1)*Fs)*1/Fs:1/Fs:ceil(data_horz(end,1)*Fs)*1/Fs)';
HorzStepResp(:,1) = bins + 1 /(2*Fs);

for i = 1:length(bins)-1
    idcs = find((data_horz(:,1) >= bins(i)) & (data_horz(:,1) < bins(i+1)));
    HorzStepResp(i,2) = mean(data_horz(idcs,2));
end

% clip the sides to smooth steprespons at the side tails with few data
% points per bin
HorzStepResp = HorzStepResp(8:end-8,:);
%[HorzStepResp(:,2), HorzStepResp(:,1)] = resample(data_horz(:,2),data_horz(:,1),Fs);

HorzImpResp = diff(HorzStepResp(:,2),1);
N = length(HorzImpResp);
w = window(@hamming,N);

Horzspect = fft(w.*HorzImpResp);
Horzfreqs = linspace(0,Fs,length(HorzImpResp));
MTF_horz  = abs(Horzspect)./abs(Horzspect(1,1));

% correction factor for discrete derivative stepresponse to obtain impulse
% response.
D= (0.25*pi()*Horzfreqs)./sin(0.25*pi().*Horzfreqs);
D = min(D,ones(size(D))*10);
D(1,1) = 1;
MTF_horz = MTF_horz.*D';

% find -3dB, 50% and 10% 
i = find(MTF_horz <= 10^(-3/20) ,1, 'first');
MTFhorz_71 = interp1(MTF_horz(i-1:i),Horzfreqs(i-1:i),10^(-3/20));
i = find(MTF_horz <= 0.5,1, 'first');
MTFhorz_50 = interp1(MTF_horz(i-1:i),Horzfreqs(i-1:i),0.5);
i = find(MTF_horz <= 0.1,1, 'first');
MTFhorz_10 = interp1(MTF_horz(i-1:i),Horzfreqs(i-1:i),0.1);

figure(1)
subplot(233)
plot(Horzfreqs,MTF_horz,'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0 0 0])
axis([0 1 0 2.5])
hold on
xlabel('frequency [cycle/px]')
ylabel('MTF [-]')
grid on
title('Modulation Transfer Function distance 30 mm')

%% Analyze the horizontal slanted edge (vertical step response)
% ROI slanted edge
Im33  = Image(Cy_33:Cy_33+Hp2,Cx_33:Cx_33+Wp2_33,:);
Im33  = imrotate(Im33,90);

% flip the image so the edge goes grom black to gray
%Im22  = fliplr(Im22);
%convert the RGB-integers to gray double
dummy    = double((Im33));
Im33gray = 0.2126*dummy(:,:,1) + 0.7152*dummy(:,:,2) + 0.0722*dummy(:,:,3);
Im33gray = (Im33gray/255).^(1/gamma)*255;

clear dummy
% obtain the number of pixels in x- and y-direction
[n,m] = size(Im33gray);


% filter image to get rid of fiber artefacts
[b,a] = butter(2,1/32,'low');
Im33gray_filt = filtfilt(b,a,Im33gray');
Im33gray_filt = Im33gray_filt';
% calculate the impuls response by diferentiating the step response
Im33diff = diff(Im33gray_filt,1,2);
% estimate x-coordinates of the edge based on the maximum values of the 
% differentiated step respons
[M Im33edge] = max(Im33diff,[],2);
% improve the estimation by fitting a straight line through the estimates
Im33_Y = [1:1:n]';
P = polyfit(Im33_Y, Im33edge+0.5,1);
Im33edgfit = polyval(P,Im33_Y);
% use meshgrid as a quick way to get a matrix containg all x-values.
[X,Y] = meshgrid([1:1:m],[1:1:n]);

% allign all impulse responses
X_edge = X-Im33edgfit;
X_edge_vertchk = X-Im33edgfit;
% change the format to resample the impulse response
X_edge = X_edge(:);
Y_edge = Im33gray(:);
data_vert = [X_edge ,Y_edge];
data_vert = sortrows(data_vert,1);
% calculate the angle of the slanted edge
alpha_vert = atan(P(1));
% transform the x-coordinates to the tilted x-coordinates
data_vert(:,1) = data_vert(:,1)./cos(alpha_vert);

% resample the impulse response to get a constant sample frequency
Fs = 4; % samples per pixel
%[VertStepResp(:,2), VertStepResp(:,1)] = resample(data_vert(:,2),data_vert(:,1),Fs);

bins = (floor(data_vert(1,1)*Fs)*1/Fs:1/Fs:ceil(data_vert(end,1)*Fs)*1/Fs)';
VertStepResp(:,1) = bins + 1 /(2*Fs);

for i = 1:length(bins)-1
    idcs = find((data_vert(:,1) >= bins(i)) & (data_vert(:,1) < bins(i+1)));
    VertStepResp(i,2) = mean(data_vert(idcs,2));
end

% clip the sides to smooth steprespons at the side tails with few data
% points per bin
VertStepResp = VertStepResp(8:end-8,:);



VertImpResp = diff(VertStepResp(:,2),1);
N = length(VertImpResp);
w = window(@hamming,N);

Vertspect = fft(w.*VertImpResp);
Vertfreqs = linspace(0,Fs,length(VertImpResp));
MTF_vert  = abs(Vertspect)./abs(Vertspect(1,1));

% correction factor for discrete derivative stepresponse to obtain impulse response
D = (0.25*pi()*Vertfreqs)./sin(0.25*pi().*Vertfreqs);
D = min(D,ones(size(D))*10);
D(1,1) = 1;
MTF_vert = MTF_vert.*D';

% find -3dB, 50% and 10% 
i = find(MTF_vert <= 10^(-3/20) ,1, 'first');
MTFvert_71 = interp1(MTF_vert(i-1:i),Vertfreqs(i-1:i),10^(-3/20));
i = find(MTF_vert <= 0.5,1, 'first');
MTFvert_50 = interp1(MTF_vert(i-1:i),Vertfreqs(i-1:i),0.5);
i = find(MTF_vert <= 0.1,1, 'first');
MTFvert_10 = interp1(MTF_vert(i-1:i),Vertfreqs(i-1:i),0.1);

figure(1)
subplot(233)
plot(Vertfreqs,MTF_vert,'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.4 0.4 0.4])
axis([0 1 0 1.1*max([MTF_horz(1:20); MTF_vert(1:20)])]);
legend('horizontal','vertical','Location','NorthEast')
%% Export results to an Excel-file
dummy={'2_4';'3_4';'4_4';'5_4';'2_5';'3_5';'4_5';'5_5';'2_6';'3_6';'4_6';'5_6'};

xlswrite([fname,'.xlsx'],{'file';'GrayAvgs'},'Sheet1','A2:A3')
xlswrite([fname,'.xlsx'],{fname},'Sheet1','B2')
xlswrite([fname,'.xlsx'],dummy,'Sheet1','A4:A15')
xlswrite([fname,'.xlsx'],GrayAvgs','Sheet1','B4:B15')
xlswrite([fname,'.xlsx'],{'GrayStds'},'Sheet1','A16')
xlswrite([fname,'.xlsx'],dummy,'Sheet1','A17:A28')
xlswrite([fname,'.xlsx'],GrayStds','Sheet1','B17:B28')
xlswrite([fname,'.xlsx'],{'Cdiffs'},'Sheet1','A29')

dummy={'2_4';'1_2';'1_3';'1_4';'1_5';'1_6';'6_2';'6_3';'6_4';'6_5';'6_6';'2_1';'3_1';'4_1';'5_1';'2_7';'3_7';'4_7';'5_7'};
xlswrite([fname,'.xlsx'],dummy,'Sheet1','A30:A48')
xlswrite([fname,'.xlsx'],Cdiffs,'Sheet1','B30:B48')
xlswrite([fname,'.xlsx'],{'MTF'},'Sheet1','A49')
dummy={'MTFhorz71';'MTFhorz50';'MTFhorz10';'MTFvert71';'MTFvert50';'MTFvert10'};
xlswrite([fname,'.xlsx'],dummy,'Sheet1','A50:A55')
MTFs = [MTFhorz_71;MTFhorz_50;MTFhorz_10;MTFvert_71;MTFvert_50;MTFvert_10];
xlswrite([fname,'.xlsx'],MTFs,'Sheet1','B50:B55')

xlswrite([fname,'.xlsx'],{'ResX [px/mm]'; 'ResY[px/mm]';'gamma'},'Sheet1','A56:A58')
xlswrite([fname,'.xlsx'],[ResX;ResY;gamma],'Sheet1','B56:B59')

xlswrite([fname,'.xlsx'],{'MTFhorz'},'Sheet1','A60')
idcsh = find(Horzfreqs<1);
xlswrite([fname,'.xlsx'],Horzfreqs(idcsh)','Sheet1',['A61:A',num2str(61+idcsh(end)-1)])
xlswrite([fname,'.xlsx'],MTF_horz(idcsh),'Sheet1',['B61:B',num2str(61+idcsh(end)-1)])
xlswrite([fname,'.xlsx'],{'MTFvert'},'Sheet1',['A',num2str(61+idcsh(end))])
idcsv = find(Vertfreqs<1);
xlswrite([fname,'.xlsx'],Vertfreqs(idcsv)','Sheet1',['A',num2str(61+idcsh(end)+1),':A',num2str(61+idcsh(end)+1+idcsv(end)-1)])
xlswrite([fname,'.xlsx'],MTF_vert(idcsv),'Sheet1',['B',num2str(61+idcsh(end)+1),':B',num2str(61+idcsh(end)+1+idcsv(end)-1)])


%% Plots for checking the horizontal edge response
[n,m] = size(Im22gray);
figure(2)
set(gcf,'Color','w')
subplot(251)
imshow(Im22)
rectangle('Position', [1,2,m,1],'EdgeColor',[0.1 0.1 0.6],'LineWidth',1.5)
rectangle('Position', [1,n-2,m,1],'EdgeColor',[0.6 0.1 0.1],'LineWidth',1.5)
dummy=uint8(round(Im22gray_filt));
dummy(:,:,3) = dummy;
dummy(:,:,2) = dummy(:,:,3);
dummy(:,:,1) = dummy(:,:,3);
subplot(252)
imshow(dummy)
subplot(253)
imshow(dummy)
hold on
plot(Im22edge,Im22_Y,'y')
subplot(254)
imshow(dummy)
hold on
plot(Im22edgfit,Im22_Y,'y')
grid on
subplot(255)
imshow(Im22)
hold on
plot(Im22edgfit,Im22_Y,'y')
grid on


subplot(256)
plot(X_edge_horzchk(:,:)+Im22edgfit,Im22gray(:,:),'.','Color',[0.6 0.6 0.6])
hold on
plot(Im22gray(2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.1 0.1 0.6])
plot(Im22gray(n-2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.6 0.1 0.1])
axis([0 m 0 200])
xlabel('pixel [#]')
ylabel('gray value [#]')

subplot(257)
line([0 0],[0 255],'Color',[0 0 0] )
hold on
plot(data_horz(:,1),data_horz(:,2),'.','LineWidth',1.5,'MarkerSize',10,'Color',[0.6 0.6 0.6])
plot(X_edge_horzchk(2,:),Im22gray(2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.1 0.1 0.6])
plot(X_edge_horzchk(n-2,:),Im22gray(n-2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.6 0.1 0.1])
axis([min(data_horz(:,1)) max(data_horz(:,1)) 0 200])
xlabel('pixel with respect to edge [#]')
ylabel('gray value [#]')


figure(2)
subplot(2,5,8)
line([0 0],[0 255],'Color',[0 0 0] )
hold on
plot(data_horz(:,1),data_horz(:,2),'.','Color',[0.6 0.6 0.6])
plot(HorzStepResp(:,1),HorzStepResp(:,2),'.-','Color',[0.8 0.1 0.1])
axis([min(data_horz(:,1)) max(data_horz(:,1)) 0 200])
xlabel('pixel with respect to edge [#]')

subplot(259)
line([0 0],[-255 255],'Color',[0 0 0] )
hold on
plot(HorzStepResp(1:end-1,1)+1/(2*Fs),HorzImpResp,'.-','Color',[0.1 0.1 0.8])
N = length(HorzImpResp);
w = window(@hamming,N);
plot(HorzStepResp(1:end-1,1)+1/(2*Fs),w.*HorzImpResp,'.-','Color',[0.8 0.1 0.1])
xlabel('pixel with respect to edge [#]')
ylabel('gray value diff [#]')
axis([min(data_horz(:,1)) max(data_horz(:,1)) -20 20])

subplot(2,5,10)
plot(Horzfreqs,MTF_horz,'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0 0 0])
%axis([0 1 0 1.1*max(abs(Horzspect))./abs(Horzspect(1,1))])
axis([0 1 0 1.6])
hold on
xlabel('frequency [cycle/pixel]')
ylabel('MTF [-]')
grid on

%% plots for checking the vertical edge response
[n,m] = size(Im33gray);
figure(3)
set(gcf,'Color','w')
subplot(251)
imshow(Im33)
rectangle('Position', [1,2,m,1],'EdgeColor',[0.1 0.1 0.6],'LineWidth',1.5)
rectangle('Position', [1,n-2,m,1],'EdgeColor',[0.6 0.1 0.1],'LineWidth',1.5)

dummy=uint8(round(Im33gray_filt));
dummy(:,:,3) = dummy;
dummy(:,:,2) = dummy(:,:,3);
dummy(:,:,1) = dummy(:,:,3);

subplot(252)
imshow(dummy) 
subplot(253)
imshow(dummy) 
hold on
plot(Im33edge,Im33_Y,'y')
subplot(254)
imshow(dummy) 
hold on
plot(Im33edgfit,Im33_Y,'y')

subplot(255)
imshow(Im33)
hold on
plot(Im33edgfit,Im33_Y,'y')
grid on


subplot(256)
plot(X_edge_vertchk(:,:)+Im33edgfit,Im33gray(:,:),'.','Color',[0.6 0.6 0.6])
hold on
plot(Im33gray(2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.1 0.1 0.6])
plot(Im33gray(n-2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.6 0.1 0.1])
axis([0 m 0 200])
xlabel('pixel [#]')
ylabel('gray value [#]')

subplot(257)
line([0 0],[0 255],'Color',[0 0 0] )
hold on
plot(data_vert(:,1),data_vert(:,2),'.','LineWidth',1.5,'MarkerSize',10,'Color',[0.6 0.6 0.6])
plot(X_edge_vertchk(2,:),Im33gray(2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.1 0.1 0.6])
plot(X_edge_vertchk(n-2,:),Im33gray(n-2,:,:),'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0.6 0.1 0.1])
axis([-m/2 m/2 0 200])
xlabel('pixel with respect to edge [#]')
ylabel('gray value [#]')

subplot(2,5,8)
line([0 0],[0 255],'Color',[0 0 0] )
hold on
plot(data_vert(:,1),data_vert(:,2),'.','Color',[0.6 0.6 0.6])
plot(VertStepResp(:,1),VertStepResp(:,2),'.-','Color',[0.8 0.1 0.1])
axis([-m/2 m/2 0 200])
xlabel('pixel with respect to edge [#]')

subplot(259)
line([0 0],[-255 255],'Color',[0 0 0] )
hold on
plot(VertStepResp(1:end-1,1)+1/(2*Fs),VertImpResp,'.-','Color',[0.1 0.1 0.8])
N = length(VertImpResp);
w = window(@hamming,N);
plot(VertStepResp(1:end-1,1)+1/(2*Fs),w.*VertImpResp,'.-','Color',[0.8 0.1 0.1])
xlabel('pixel with respect to edge [#]')
ylabel('gray value diff [#]')
axis([-m/2 m/2 -20 20])

subplot(2,5,10)
plot(Vertfreqs,MTF_vert,'.-','LineWidth',1.5,'MarkerSize',10,'Color',[0 0 0])
axis([0 1 0 1.1*max(abs(Vertspect))./abs(Vertspect(1,1))])
hold on
xlabel('frequency [cycle/pixel]')
ylabel('MTF [-]')
grid on