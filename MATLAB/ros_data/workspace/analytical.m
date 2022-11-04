L=Beam_width*L_by_width;
E=youngsModulus;
I=Beam_width^4/12;
Li=L/nSection;
Q=sym('q',[nSection 1]);
%% finding x and y in terms of Qs
x=0;
y=0;
theta=0;
for i=1:nSection
    X=[cos(theta) -sin(theta);sin(theta) cos(theta)]*[Li*(1-cos(Q(i)))/Q(i);Li*sin(Q(i))/Q(i)];
    theta=theta+Q(i);
    x=x+X(1,1);
    y=y+X(2,1);
end
%% 
X=simplify([x;y]);
jacob_=jacobian(X,Q);
M=tan(Q)*E*I/L;
final_matrix=simplify(jacob_/(jacobian(M,Q)));
%% 
step_t=zeros(6^5,2,5);
tipPose_t=zeros(6^5,2,1);
%%
count=0;
for q1=-pi/2:0.6:pi/2
    for q2=-pi/2:0.6:pi/2
        for q3=-pi/2:0.6:pi/2
            for q4=-pi/2:0.6:pi/2
                for q5=-pi/2:0.6:pi/2
                    count=count+1;
                    tipPose_t(count,:,:)=double(subs(X,Q,[q1;q2;q3;q4;q5]));
                    step_t(count,:,:)=double(subs(final_matrix,Q,[q1;q2;q3;q4;q5]));
                end
            end
            count
        end
    end
end
%%
step=step_t(3686,:);
step_matrix=zeros(2,5);
step_matrix(1,:)=step(1,1,:);
step_matrix(2,:)=step(1,2,:);
[u,sigma,v]=svd(step_matrix);
    