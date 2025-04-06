function h = plot_error(x, y1, y2, x_label_latex, y_label_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 1;

h = figure('Renderer', 'painters', 'Position', [10 10 900 350]);
removeToolbarExplorationButtons(h)

for i=1:size(x)
    y(i) = y1(i) - y2(i);
end

plot(x, y, 'k-', 'Linewidth', lw ,'Color', [1, 0, 0]);

xlabel(x_label_latex)
ylabel(y_label_latex)
set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');
exportgraphics(h, pdf_name);

end