function [freq1,freq2]=OurFFT(inp)

A_S=inp;
B=inp;
for i=1:9
    A_S(i)= sum(B(1:i))/i;
end
for i=10:length(B)
    A_S(i)= sum(B(i-9:i))/10;
end
%A_S=A_S(700:end);

%sampling freq
Fs=100;
T=1/Fs;

%figure
%plot(A_S);
Y=fft(A_S);

%figure;
%positive values
P2=abs(Y);
L=length(P2);
P1=P2(1:L/2+1);
P1(2:end-1)=2*P1(2:end-1);
f=Fs*(0:L/2)/L;
m=plot(f,P1);
q=get(m,'ydata');
r=get(m,'xdata');

%q=q(2:end);
cutoff_left=find(r==1);
cutoff_right=find(r==2.5);
r=r(cutoff_left+1:cutoff_right);
q=q(cutoff_left+1:cutoff_right);
[v1,idx1]=max(q);
[v2,idx2]=max(q(q<max(q)));
freq1=r(idx1);
freq2=r(idx2);

    fprintf("The component freq1 = %d\n",freq1*60);
    fprintf("The component freq2 = %d\n",freq2*60);

end