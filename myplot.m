function myplot(A, B, C, N, theta_a, theta_b, theta_c)
figure
plot(1:N, repmat(A(2:end), N, 1), '--')
hold on
grid on
plot(1:N,theta_a)
xlabel('迭代次数k'), ylabel('A(k)')
xlim([0,150])

figure
plot(1:N, repmat(B, N, 1), '--')
hold on
grid on
plot(1:N, theta_b)
xlabel('迭代次数k'), ylabel('B(k)')
xlim([0,150])

if length(C)>1
    figure
    plot(1:N, repmat(C(2:end), N, 1), '--')
    hold on
    grid on
    plot(1:N, theta_c)
    xlabel('迭代次数k'), ylabel('C(k)')
    xlim([0,150])
end
end