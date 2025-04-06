function h = plot_velocities(x1, x2, y1, y2, x_label_latex, y1_label_latex, y2_label_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 1;

h = figure('Renderer', 'painters', 'Position', [10 10 900 500]);
removeToolbarExplorationButtons(h)

% First subplot
subplot(2, 1, 1);
plot(x1, y1, 'k-', 'LineWidth', lw, 'Color', [1, 0, 0]);
xlabel(x_label_latex);
ylabel(y1_label_latex);
set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');

% Second subplot
subplot(2, 1, 2);
plot(x2, y2, 'k-', 'LineWidth', lw, 'Color', [0, 0, 1]);
xlabel(x_label_latex);
ylabel(y2_label_latex);

set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');
exportgraphics(h, pdf_name);

end