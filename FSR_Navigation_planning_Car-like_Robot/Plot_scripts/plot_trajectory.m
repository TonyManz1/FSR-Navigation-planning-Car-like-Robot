function h = plot_trajectory(x, y, x_label_latex, y_label_latex, pdf_name)
 
    set(0, 'DefaultTextInterpreter', 'latex')
    set(0, 'DefaultLegendInterpreter', 'latex')
    set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
    
    lw = 1;
    h = figure('Renderer', 'painters', 'Position', [10 10 900 700]); 
    
    removeToolbarExplorationButtons(h);
    
    plot(x, y, 'k-', 'Linewidth', lw, 'Color', [1, 0, 0]);
    
    xlabel(x_label_latex);
    ylabel(y_label_latex);
    
    set(gca, 'FontSize', 18);    
    grid on;
    box on;
    axis equal;
    
    xlim([0 450]);    
    ylim([80 180]);   % [40 180] for RRT*
    
    set(gcf, 'color', 'w');
    exportgraphics(h, pdf_name);
end