close all; clc; clear;
addpath(genpath('./arrow3d'))
% [x,y,z] = meshgrid(-2:1:2,-2:1:2,-2:1:2);
% x = x(:);
% y = y(:);
% z = z(:);
% 
figure; hold on;

V=rand(3,20);
x=V(1,:);
y=V(2,:);
z=V(3,:);

[k1,av1] = convhull(x,y,z);

trisurf(k1,x,y,z,'FaceColor','cyan')
k_all=k1(:);
k_all_unique=unique(k1);

for i=1:length(k_all_unique)
    plotSphere([x(k_all_unique(i)) y(k_all_unique(i)) z(k_all_unique(i))], 0.03, 'b')
end
axis equal

camlight
lighting phong

V=rand(3,20);
x=V(1,:)+1.5;
y=V(2,:)+1.5;
z=V(3,:);

[k1,av1] = convhull(x,y,z);


trisurf(k1,x,y,z,'FaceColor','red')
k_all=k1(:);
k_all_unique=unique(k1);

for i=1:length(k_all_unique)
    plotSphere([x(k_all_unique(i)) y(k_all_unique(i)) z(k_all_unique(i))], 0.03, 'm')
end




A=1;
B=1
C=1;
D=-3.2;
[x y] = meshgrid(0.5:0.1:2); % Generate x and y data
z = -1/C*(A*x + B*y + D); % Solve for z data
surf(x,y,z,'FaceAlpha',0.5) %Plot the surface
view([45,45])
axis equal;  axis off; plotAxesArrows(0.6);


exportAsSvg(gcf,'polyhedra')

function handle=plotSphere(position, radius, color)

    [x,y,z] = sphere(50);
    handle = surf(x*radius+position(1),y*radius+position(2),z*radius+position(3),'FaceColor',color,'LineStyle','none' );

end

function plotAxesArrows(length)
arrow3d([0 0 0],[0 0 length],20,'cylinder',[0.2,0.1]);
arrow3d([0 0 0],[0 length 0],20,'cylinder',[0.2,0.1]);
arrow3d([0 0 0],[length 0 0],20,'cylinder',[0.2,0.1]);
end

function exportAsSvg(handle, name_figure)

set(handle,'Units','inches');
screenposition = get(handle,'Position');
set(gcf,'PaperPosition',[0 0 screenposition(3:4)],'PaperSize',[screenposition(3:4)]);
print (name_figure, '-dsvg', '-painters')
set(gcf,'renderer','Painters')
% system(['pdfcrop ',name_figure,'.svg ',name_figure,'.pdf']);

end