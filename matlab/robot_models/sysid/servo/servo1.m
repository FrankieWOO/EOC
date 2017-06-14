
s = serial('COM7','BaudRate',115200);
%s.ReadAsyncMode = 'manual';
fopen(s);

u1 = 1250;
message2write = ['#' num2str(u1) '*'];
fprintf(s,message2write);

%%
%%readasync(s)

if(s.BytesAvailable>0)
    
   fscanf(s); 
    
end

%%
