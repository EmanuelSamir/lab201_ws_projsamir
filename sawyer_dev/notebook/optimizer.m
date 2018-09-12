x0 = 0;
x1 = 0 ; 
a1 = 0.4;
a2 = 0.02;
fa = 0:0.01:5;
fb = 0:0.01:10;
t_1 = 0:0.01:10;
t = ((t_1(2)- t_1(1))/2)*randn(1,length(t_1)) + t_1;

xk = @(t, f1, f2) a1*sin(f1*(t+ a2 * sin(f2*t)));
t0 =0;      
t1 = 0;
V = [];
del = 0;
D = [] ;
Data = [];
n_rate = randn(1,1);
for f1 = fa
    for f2 =fb
        for i = t
            t1 = i;
            del = xk(t1, f1, f2) - xk (t0, f1, f2);
            x1 = x0 + del;
            D = [D del];
            V = [V x1];
            x0 = x1;
            t0 = t1;
        end
        X = norm(V - a1 * sin (f1*t_1));
        Y = norm(D - a2 * sin (f2*t_1));
        Data = [Data; f1, f2, X, Y];
        D = [];
        V = [];
    end
end
