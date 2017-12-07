s=serial('COM4','BaudRate',115200);
fopen(s);
%get(s)
set(s,'Terminator','CR');

line='a';

data=zeros(1,2500);
f1a=[];
f2a=[];
j=0;
B=[];
while(1)
    flushinput(s);
    for(i=1:800)
        A = fscanf(s,'%d');
        
        B=[B,A];
    end
    %figure
    %plot(1:length(B),B);
    [f1,f2]=OurFFT(B);
    f1a=[f1a,max(f1,f2)*60];
    f2a=[f2a,min(f1,f2)*60];
    if(j>9)
        j=0;
        avg1=sum(f1a)/10
        avg2=sum(f2a)/10
        f1a=[];
        f2a=[];
    end
    j=j+1
    B=[];
    %fprintf(s,'%s\n','Hi:D');
    
end

fclose(s);
delete(s);
%fclose(instrfind);