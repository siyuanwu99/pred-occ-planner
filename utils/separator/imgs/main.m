close all; clc; clear;
set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
addpath(genpath('./arrow3d'))
set(0,'defaultfigurecolor',[1 1 1])
% [x,y,z] = meshgrid(-2:1:2,-2:1:2,-2:1:2);
% x = x(:);
% y = y(:);
% z = z(:);
% 
figure; hold on;
V=rand(3,20);

Vb=[

    0.4189    0.8640    0.0708    0.1529    0.9591    0.0128    0.8074    0.9024    0.7910    0.3000    0.7926    0.2534    0.0247    0.4506    0.4984    0.6416    0.4979    0.5364    0.7940    0.3678;
    0.1271    0.2746    0.3788    0.6310    0.4987    0.6054    0.6550    0.1522    0.0607    0.7342    0.7827    0.0710    0.0620    0.6723    0.0488    0.7864    0.8184    0.3309    0.3432    0.6796;
    0.6546    0.8402    0.2682    0.3164    0.7386    0.5765    0.8782    0.1926    0.3898    0.1042    0.5324    0.6258    0.1296    0.8561    0.3138    0.2892    0.5951    0.4117    0.4626    0.5678;    

]

% V=rand(3,20);
Va=[
    2.4433    1.6138    1.5846    1.8281    1.7537    1.6577    1.6078    1.9274    1.9474    2.2731    1.9064    1.6275    2.0786    1.5331    1.8583    2.0116    1.5924    1.5966    1.5113    1.8851;
    1.7898    2.4649    2.2167    2.2535    2.0344    2.1005    2.4000    1.6524    2.0328    2.3817    2.1042    1.9962    2.4436    2.4294    1.7600    2.0625    2.3726    2.3459    2.0237    2.1493;
    0.3766    0.4325    0.5068    0.8360    0.4352    0.9375    0.5505    0.2475    0.3547    0.7341    0.6411    0.3105    0.4269    0.9250    0.7869    0.6848    0.9429    0.9094    0.6503    0.7629;
];

plotPolyhedron(Va,'red')
plotPolyhedron(Vb,'cyan')

axis equal; camlight  
lighting phong

%%
interval=[0.0 2 0.0 2 0 2];
n1=[1 1 1]';
d1=-2.9;
figure;
subplot(2,2,1); hold on;view([45,45]); axis equal;  axis off; plotAxesArrows(0.6);
camlight
lighting phong
plotPolyhedron(Va,'red')
plotPolyhedron(Vb,'cyan')

% f = @(x,y,z) n'*[x;y;z]+d;
fimplicit3( @(x,y,z) n1'*[x;y;z]+d1,interval,'FaceColor','g')


subplot(2,2,2); hold on;view([45,45]); axis equal;  axis off; plotAxesArrows(0.6);
camlight
lighting phong
plotPolyhedron(Va,'red')
plotPolyhedron(Vb,'cyan')

epsilon=0.3;

n=n1/epsilon; 
d=d1/epsilon;
% 
fimplicit3( @(x,y,z) n1'*[x;y;z]+d1,interval,'FaceColor','g')
fimplicit3( @(x,y,z) n'*[x;y;z]+d-1,interval,'FaceColor','r')
fimplicit3( @(x,y,z) n'*[x;y;z]+d+1,interval,'FaceColor','b')


subplot(2,2,3);
hold on;view([45,45]); axis equal;  axis off; plotAxesArrows(0.6);
camlight
lighting phong
plotPolyhedron(Va,'red')
plotPolyhedron(Vb,'cyan')

delta_min=inf;
for i=1:size(Va,2)
    point=Va(:,i);
    delta_min=min(delta_min,abs((n'*point+d-1)/norm(n)));
end 

distance_red_to_blue=2/norm(n);


fimplicit3( @(x,y,z) n'*[x;y;z]+(d-norm(n)*delta_min)-1,interval,'FaceColor','r')
fimplicit3( @(x,y,z) n'*[x;y;z]+(d-norm(n)*delta_min)+1,interval,'FaceColor','b')


subplot(2,2,4);
hold on;view([45,45]); axis equal;   axis off; plotAxesArrows(0.6);
camlight
lighting phong
plotPolyhedron(Va,'red')
plotPolyhedron(Vb,'cyan')


alpha=0.9;

fimplicit3( @(x,y,z) n'*[x;y;z]+(d-norm(n)*delta_min)-1,interval,'FaceColor','r')
fimplicit3( @(x,y,z) n'*[x;y;z]+(d-norm(n)*delta_min-2*alpha)+1,interval,'MeshDensity',50,'FaceColor','b')
% fimplicit3( @(x,y,z) n2'*[x;y;z]+(d2-norm(n2)*delta_min-factor_tmp*norm(n2)*distance_red_to_blue)+1,interval,'MeshDensity',50,'FaceColor','b')

% exportAsSvg(gcf,'polyhedra')

function plotPolyhedron(V,color)

x=V(1,:);
y=V(2,:);
z=V(3,:);

[k1,av1] = convhull(x,y,z);

trisurf(k1,x,y,z,'FaceColor',color)
k_all=k1(:);
k_all_unique=unique(k1);

for i=1:length(k_all_unique)
    plotSphere([x(k_all_unique(i)) y(k_all_unique(i)) z(k_all_unique(i))], 0.03, color)
end

end



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