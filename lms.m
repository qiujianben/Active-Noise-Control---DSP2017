function [Shy,Shw]= lms(Shx,y_iden,x_iden,Shw,e_iden,T)

mu=0.1;                         % learning rate
for k=1:T,                      % discrete time k
    Shx=[x_iden(k) Shx(1:15)];  % update the state
    Shy=sum(Shx.*Shw);	        % calculate output of Sh(z)
    e_iden(k)=y_iden(k)-Shy;    % calculate error         
    Shw=Shw+mu*e_iden(k)*Shx;   % adjust the weight
end

end