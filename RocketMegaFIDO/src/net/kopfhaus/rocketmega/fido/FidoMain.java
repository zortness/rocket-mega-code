package net.kopfhaus.rocketmega.fido;

import net.kopfhaus.rocketmega.fido.ui.FidoMainForm;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PiePlot;
import org.jfree.data.general.DefaultPieDataset;
import org.jfree.util.Rotation;

import javax.swing.*;
import java.awt.*;

/**
 * Main entry point for the RocketMegaFido project.
 * @author Kurtis Kopf
 */
public class FidoMain extends JFrame implements Runnable
{
	public static void main(String[] args)
	{
		SwingUtilities.invokeLater(new FidoMain());
	}

	public FidoMain()
	{
	}

	@Override
	public void run()
	{
		System.out.println("FIDO starting up...");
		initUi();
		this.pack();
		this.setVisible(true);
	}

	private void initUi()
	{
		FidoMainForm main = new FidoMainForm();
		getContentPane().add(main.panel1);

		DefaultPieDataset pieDataset = new DefaultPieDataset();
		pieDataset.setValue("Linux", 29);
		pieDataset.setValue("Mac", 20);
		pieDataset.setValue("Windows", 51);
		JFreeChart chart = ChartFactory.createPieChart("Test Chart", pieDataset, true, true, false);
		PiePlot plot = (PiePlot)chart.getPlot();
		plot.setStartAngle(290);
		plot.setDirection(Rotation.CLOCKWISE);
		plot.setForegroundAlpha(0.5f);
		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setPreferredSize(new Dimension(500,270));

		main.graphPanel.removeAll();
		main.graphPanel.setLayout(new FlowLayout(FlowLayout.CENTER));
		main.graphPanel.setMinimumSize(new Dimension(500,270));
		main.graphPanel.add(chartPanel);


		setTitle("RocketMega FIDO");
		setSize(300, 200);
		setLocationRelativeTo(null);
		setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
	}
}
