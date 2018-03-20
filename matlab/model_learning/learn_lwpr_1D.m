
clear all;clf;
n = 10000;
% training dataset
X = rand(n,1);
Y = exp(X.^2) + rand(n,1)*0.1; 

% test dataset
Xt = 0:0.02:1;
Xt = Xt';
Yt = exp(Xt.^2); 



tic
% initialize LWPR
model = lwpr_init(1,1,'name','lwpr_test');

model = lwpr_set(model,'init_D',1000000);  
model = lwpr_set(model,'update_D',0);
model = lwpr_set(model,'init_alpha',250);
model = lwpr_set(model,'w_gen',0.1);
model = lwpr_set(model,'diag_only',0);   
model = lwpr_set(model,'meta',1);
model = lwpr_set(model,'meta_rate',250);
model = lwpr_set(model,'kernel','Gaussian');   

%%%%%%%%%%%%%%%%%%%%%%%%
%  Transfer model into mex-internal storage
   model = lwpr_storage('Store',model);
%%%%%%%%%%%%%%%%%%%%%%%%

% train the model
for j=1:20
   inds = randperm(n);

   mse = 0;
   for i=1:n,
	   [model,yp,w] = lwpr_update(model,X(inds(i),:)',Y(inds(i),:)');         
	   mse = mse + (Y(inds(i),:)-yp).^2;
   end

   nMSE = mse/n/var(Y,1);
   fprintf(1,'#Data=%d #rfs=%d nMSE=%5.3f\n',lwpr_num_data(model),lwpr_num_rfs(model),nMSE);
   if exist('fflush') % for Octave output only
      fflush(1);
   end   
end


% create predictions for the test data
Yp = zeros(size(Yt));
Conf = zeros(size(Yt));
for i=1:length(Xt),
	[Yp(i,1),Conf(i,1)]=lwpr_predict(model,Xt(i,:)',0.001);
end
%[yp,w]=lwpr_predict(model,Xt',0.001);
%Yp = yp';

ep   = Yt-Yp;
mse  = mean(ep.^2);
nmse = mse/var(Yt,1);

toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Transfer model back from mex-internal storage
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = lwpr_storage('GetFree',model);



plot(Xt,Yt,'b-');
hold on
plot(Xt,Yp,'r.');
plot(Xt,Yp+Conf,'c--');
plot(Xt,Yp-Conf,'c--');

t = linspace(0,2*pi,50);
ct = cos(t);
st = sin(t);

for k=1:length(model.sub(1).rfs); 
   r = 1/sqrt(model.sub(1).rfs(k).D);
   x = model.sub.rfs(k).c;
   y = model.sub.rfs(k).beta0;
   b = model.sub.rfs(k).beta * model.sub.rfs(k).U;
   plot(x+r*ct,r*st,'g-');
   plot([x-r,x+r],[y-r*b,y+r*b],'k-');
end

