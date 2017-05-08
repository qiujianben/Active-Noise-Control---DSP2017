
% We do not know P(z) and S(z) in reality. So we have to make dummy paths
Pw=[0.01 0.25 0.5 1 0.5 0.25 0.01];
Sw=Pw*0.25;

filename = 'television.wav';
%audiowrite(filename, data, fs);

%player = audioplayer(data,fs);
%play(player);

noise = rand(size(data));

X = data;

figure
plot(data);
title('Input Data .wav file');
figure
plot(noise(1:1000));
title('Noise data');

T=length(X); 

x_iden=randn(1,T);

% send it to the actuator, and measure it at the sensor position, 
y_iden=filter(Sw, 1, x_iden);

% Then, start the identification process
Shx=zeros(1,16);     % the state of Sh(z)
Shw=zeros(1,16);     % the weight of Sh(z)
e_iden=zeros(1,T);   % data buffer for the identification error

% and apply least mean square algorithm
mu=0.1;                         % learning rate
for k=1:T,                      % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error         
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
end

% Lets check the result
figure
subplot(2,1,1)
plot([1:T], e_iden)
ylabel('Amplitude');
xlabel('Discrete time k');
legend('Identification error');
subplot(2,1,2)
stem(Sw) 
hold on 
stem(Shw, 'r*')
ylabel('Amplitude');
xlabel('Numbering of filter tap');
legend('Coefficients of S(z)', 'Coefficients of Sh(z)')


% The second task is the active control itself. Again, we need to simulate 
% the actual condition. In practice, it should be an iterative process of
% 'measure', 'control', and 'adjust'; sample by sample. Now, let's generate 
% the noise: 
%X=randn(1,T);

% and measure the arriving noise at the sensor position,
Yd=filter(Pw, 1, X);
  
% Initiate the system,
Cx=zeros(1,16);       % the state of C(z)
Cw=zeros(1,16);       % the weight of C(z)
Sx=zeros(size(Sw));   % the dummy state for the secondary path
e_cont=zeros(1,T);    % data buffer for the control error
Xhx=zeros(1,16);      % the state of the filtered x(k)

% and apply the FxLMS algorithm
mu=0.1;                            % learning rate
for k=1:T,                         % discrete time k
    Cx=[X(k) Cx(1:15)];            % update the controller state    
    Cy=sum(Cx.*Cw);                % calculate the controller output	
    Sx=[Cy Sx(1:length(Sx)-1)];    % propagate to secondary path
    e_cont(k)=Yd(k)-sum(Sx.*Sw);   % measure the residue
    Shx=[X(k) Shx(1:15)];          % update the state of Sh(z)
    Xhx=[sum(Shx.*Shw) Xhx(1:15)]; % calculate the filtered x(k)
    Cw=Cw+mu*e_cont(k)*Xhx;        % adjust the controller weight
end


% Report the result
figure
subplot(2,1,1)
plot([1:T], e_cont)
ylabel('Amplitude');
xlabel('Discrete time k');
title('Noise Separation');
subplot(2,1,2)
plot([1:T], Yd) 
 hold on 
plot([1:T], Yd-e_cont, 'r:')
ylabel('Amplitude');
xlabel('Discrete time k');

