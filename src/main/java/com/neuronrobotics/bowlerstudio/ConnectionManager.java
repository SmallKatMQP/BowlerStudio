package com.neuronrobotics.bowlerstudio;

import com.neuronrobotics.addons.driving.HokuyoURGDevice;
import com.neuronrobotics.bowlerstudio.assets.AssetFactory;
import com.neuronrobotics.bowlerstudio.utils.BowlerConnectionMenu;
import com.neuronrobotics.imageprovider.AbstractImageProvider;
//import com.neuronrobotics.imageprovider.OpenCVImageProvider;
import com.neuronrobotics.imageprovider.StaticFileProvider;
import com.neuronrobotics.imageprovider.URLImageProvider;
import com.neuronrobotics.sdk.addons.gamepad.BowlerJInputDevice;
import com.neuronrobotics.sdk.addons.gamepad.IJInputEventListener;
import com.neuronrobotics.sdk.addons.kinematics.FirmataBowler;
import com.neuronrobotics.sdk.addons.kinematics.gcodebridge.GcodeDevice;
import com.neuronrobotics.sdk.common.*;
import com.neuronrobotics.sdk.javaxusb.UsbCDCSerialConnection;
import com.neuronrobotics.sdk.network.BowlerTCPClient;
import com.neuronrobotics.sdk.network.UDPBowlerConnection;
import com.neuronrobotics.sdk.serial.SerialConnection;
import com.neuronrobotics.sdk.ui.AbstractConnectionPanel;
import com.neuronrobotics.sdk.util.ThreadUtil;
import com.neuronrobotics.sdk.wireless.bluetooth.BluetoothSerialConnection;
import gnu.io.NRSerialPort;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.control.*;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.text.Text;
import javafx.stage.FileChooser;
import javafx.stage.Modality;
import javafx.stage.Stage;
import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import org.reactfx.util.FxTimer;

import java.io.File;
import java.net.MalformedURLException;
import java.net.URL;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

//import org.bytedeco.javacv.OpenCVFrameGrabber;

public class ConnectionManager extends Tab implements IDeviceAddedListener ,EventHandler<ActionEvent> {

	private static VBox rootItem;
	private static final ArrayList<PluginManagerWidget> plugins = new ArrayList<PluginManagerWidget>();
	//private BowlerStudioController bowlerStudioController;
	String formatStr="%1$-40s %2$-60s  %3$-40s";
	private static final ConnectionManager connectionManager;
	private static Button disconnectAll;
	private static HBox topLine;
	final static Accordion accordion = new Accordion (); 
	static{
		connectionManager = new ConnectionManager();
	}


	private Node getIcon(String s) {
		return new ImageView(new Image(
				AbstractConnectionPanel.class.getResourceAsStream(s)));
	}

	public ConnectionManager() {
		if(connectionManager!=null){
			throw new RuntimeException("Connection manager is a static singleton, access it using ConnectionManager.getConnectionmanager()");
		}
		setText("My Devices");
		setGraphic(AssetFactory.loadIcon("My-Devices.png"));
		rootItem = new VBox(10);
		
//		rootItem.getColumnConstraints().add(new ColumnConstraints(30)); // column 1 is 75 wide
//		rootItem. getColumnConstraints().add(new ColumnConstraints(80)); // column 2 is 300 wide
//		rootItem.getColumnConstraints().add(new ColumnConstraints(100)); // column 2 is 100 wide
//		rootItem.getColumnConstraints().add(new ColumnConstraints(50)); // column 2 is 100 wide
//		
		topLine = new HBox(20);
		
		disconnectAll = new Button("Disconnect All",AssetFactory.loadIcon("Disconnect-All.png"));
		disconnectAll.setOnAction(new EventHandler<ActionEvent>() {
		    @Override public void handle(ActionEvent e) {
		    	disconnectAll();
		    }
		});
		disconnectAll.setDisable(true);
		topLine.getChildren().addAll(AssetFactory.loadIcon("Connected-Devices.png"),new Text("Connected Devices"),disconnectAll);
		rootItem.getChildren().add(topLine);

		rootItem.getChildren().add(accordion);
//		rootItem = new CheckBoxTreeItem<String>( String.format("  "+formatStr, "SCRIPTING NAME","DEVICE TYPE","MAC ADDRESS"),
//				getIcon("images/connection-icon.png"
//				// "images/usb-icon.png"
//				));
//		rootItem.setExpanded(true);
//		rootItem.setSelected(true);
//		rootItem.selectedProperty().addListener(b -> {
//			if (!rootItem.isSelected()) {
//				disconnectAll();
//			}
//		});
		

		setContent(rootItem);
		
		DeviceManager.addDeviceAddedListener(this);
			

	}



	public static void addConnection(BowlerAbstractDevice newDevice, String name) {


		DeviceManager.addConnection(newDevice, name);
		
	}

	@Override
	public void handle(ActionEvent event) {
		// TODO Auto-generated method stub

	}

	public static ArrayList<PluginManagerWidget>  getPlugins() {
		return plugins;
	}
//
//	public BowlerStudioController getBowlerStudioController() {
//		return bowlerStudioController;
//	}
//
//	public void setBowlerStudioController(
//			BowlerStudioController bowlerStudioController) {
//		this.bowlerStudioController = bowlerStudioController;
//	}
	
	public static BowlerAbstractDevice pickConnectedDevice(Class class1) {
		List<String> choices = DeviceManager.listConnectedDevice(class1);
		
		if(!choices.isEmpty()){
			ChoiceDialog<String> dialog = new ChoiceDialog<>(choices.get(0),
					choices);
			dialog.setTitle("Bowler Device Chooser");
			dialog.setHeaderText("Choose connected bowler device");
			dialog.setContentText("Device Name:");
	
			// Traditional way to get the response value.
			Optional<String> result = dialog.showAndWait();
			if (result.isPresent()) {
				for (int i = 0; i < plugins.size(); i++) {
					if (plugins.get(i).getManager().getName().contains(result.get())) {
						return plugins.get(i).getManager().getDevice();
					}
				}
			}
		}else{
			Alert alert = new Alert(AlertType.INFORMATION);
			alert.setTitle("Device not availible");
			alert.setHeaderText("Connect a "+class1.getSimpleName());
			alert.setContentText("A device of type "+class1.getSimpleName()+" is needed");
			alert .initModality(Modality.APPLICATION_MODAL);
			alert.show();
		}
		return null;
	}


//	public BowlerAbstractDevice pickConnectedDevice() {
//
//		return pickConnectedDevice(null);
//	}

	public static void disconnectAll() {

		//extract list int thread safe object
		Object [] pms= plugins.toArray();
		for (int i=0;i<pms.length;i++) {
			disconectAndRemoveDevice(((PluginManagerWidget)pms[i]).getManager());
			ThreadUtil.wait(50);
		}

	}
	
//	 public static OpenCVImageProvider onConnectCVCamera() {
//		List<String> choices = new ArrayList<>();
//		choices.add("0");
//		choices.add("1");
//		choices.add("2");
//		choices.add("3");
//		choices.add("4");
//		
//		ChoiceDialog<String> dialog = new ChoiceDialog<>("0", choices);
//		dialog.setTitle("OpenCV Camera Index Chooser");
//		dialog.setHeaderText("Choose an OpenCV camera");
//		dialog.setContentText("Camera Index:");
//
//		// Traditional way to get the response value.
//		Optional<String> result = dialog.showAndWait();
//		
//		// The Java 8 way to get the response value (with lambda expression).
//		if (result !=null) {
//			String letter = result.get();
//			OpenCVImageProvider p = new OpenCVImageProvider(Integer.parseInt(letter));
//			String name = "camera"+letter;
//			addConnection(p,name);
//			return p;
//		}
//		return null;
////		OpenCVImageProvider p = new OpenCVImageProvider(0);
////		String name = "camera0";
////		application.addConnection(p,name);
//		
//	}
//


	 public static void onConnectJavaCVCamera() {
		// onConnectCVCamera();
//		List<String> choices = new ArrayList<>();
//		try {
//			String[] des = OpenCVFrameGrabber.getDeviceDescriptions();
//			if(des.length==0)
//				return;
//			for (String s: des){
//				choices.add(s);
//			}
//		} catch (org.bytedeco.javacv.FrameGrabber.Exception |UnsupportedOperationException e1) {
//			choices.add("0");
//			choices.add("1");
//			choices.add("2");
//			choices.add("3");
//			choices.add("4");
//		}
//		
//		ChoiceDialog<String> dialog = new ChoiceDialog<>(choices.get(0), choices);
//		dialog.setTitle("JavaCV Camera Index Chooser");
//		dialog.setHeaderText("Choose an JavaCV camera");
//		dialog.setContentText("Camera Index:");
//
//		// Traditional way to get the response value.
//		Optional<String> result = dialog.showAndWait();
//		
//		// The Java 8 way to get the response value (with lambda expression).
//		result.ifPresent(letter -> {
//			JavaCVImageProvider p;
//			try {
//				p = new JavaCVImageProvider(Integer.parseInt(letter));
//				String name = "camera"+letter;
//				addConnection(p,name);
//			} catch (Exception e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//
//		});
//		

	}


	 public static void onConnectFileSourceCamera() {
		 Platform.runLater(()->{});
		 
		FileChooser fileChooser = new FileChooser();
		fileChooser.setTitle("Open Image File");
		File f = fileChooser.showOpenDialog(BowlerStudioModularFrame.getPrimaryStage());
		if(f!=null){
			AbstractImageProvider p = new StaticFileProvider(f);
			String name = "image";
			addConnection(p,name);
		}
	}


	public static void onConnectURLSourceCamera() {
		TextInputDialog dialog = new TextInputDialog("http://neuronrobotics.com/img/AndrewHarrington/2014-09-15-86.jpg");
		dialog.setTitle("URL Image Source");
		dialog.setHeaderText("This url will be loaded each capture.");
		dialog.setContentText("URL ");

		// Traditional way to get the response value.
		Optional<String> result = dialog.showAndWait();
		if (result.isPresent()){
			URLImageProvider p;
			try {
				p = new URLImageProvider(new URL(result.get()));
				String name = "url";
				addConnection(p,name);
			} catch (MalformedURLException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}

	 public static void onMarlinGCODE() {
			Set<String> ports = NRSerialPort.getAvailableSerialPorts();
			List<String> choices = new ArrayList<>();
			if(ports.isEmpty())
				return;
			for (String s: ports){
				choices.add(s);
			}

			
			ChoiceDialog<String> dialog = new ChoiceDialog<>(choices.get(0), choices);
			dialog.setTitle("GCODE Device Serial Port Chooser");
			dialog.setHeaderText("Supports Marlin");
			dialog.setContentText("GCODE Device Port:");

			// Traditional way to get the response value.
			Optional<String> result = dialog.showAndWait();
			
			// The Java 8 way to get the response value (with lambda expression).
			result.ifPresent(letter -> {
				GcodeDevice p =  new GcodeDevice(new NRSerialPort(letter, 115200));
				p.connect();
				String name = "GCODE";
				addConnection(p,name);
			});
			
		}
	 public static void onConnectHokuyoURG() {
		Set<String> ports = NRSerialPort.getAvailableSerialPorts();
		List<String> choices = new ArrayList<>();
		if(ports.isEmpty())
			return;
		for (String s: ports){
			choices.add(s);
		}

		
		ChoiceDialog<String> dialog = new ChoiceDialog<>(choices.get(0), choices);
		dialog.setTitle("LIDAR Serial Port Chooser");
		dialog.setHeaderText("Supports URG-04LX-UG01");
		dialog.setContentText("Lidar Port:");

		// Traditional way to get the response value.
		Optional<String> result = dialog.showAndWait();
		
		// The Java 8 way to get the response value (with lambda expression).
		result.ifPresent(letter -> {
			HokuyoURGDevice p = new HokuyoURGDevice(new NRSerialPort(letter, 115200));
			p.connect();
			String name = "lidar";
			addConnection(p,name);
		});
		
	}


	public static void onConnectGamePad(String name ) {
		Controller[] ca = ControllerEnvironment.getDefaultEnvironment().getControllers();
		
		List<String> choices = new ArrayList<>();
		if(ca.length==0)
			return;
		for (Controller s: ca){
			choices.add(s.getName());
		}

		
		ChoiceDialog<String> dialog = new ChoiceDialog<>(choices.get(0), choices);
		dialog.setTitle("JInput Game Controller Select");
		dialog.setHeaderText("Connect a game controller");
		dialog.setContentText("Controller:");

		// Traditional way to get the response value.
		Optional<String> result = dialog.showAndWait();
		
		// The Java 8 way to get the response value (with lambda expression).
		result.ifPresent(letter -> {
			for(Controller s: ca){
				if(letter.contains(s.getName())){
					BowlerJInputDevice p =new BowlerJInputDevice(s);
					p.connect();
					IJInputEventListener l=new IJInputEventListener() {
						@Override
						public void onEvent(Component comp, Event event1,
								float value, String eventString) {
									//System.out.println(comp.getName()+" is value= "+value);
								}
					};
					p.addListeners(l);
					addConnection(p,name);
					return;
				}
			}

		});
		
	}

	@Override
	public void onNewDeviceAdded(BowlerAbstractDevice newDevice) {
		
		PluginManager mp;
		Log.debug("Adding a "+newDevice.getClass().getName()+" with name "+newDevice.getScriptingName() );
		mp = new PluginManager(newDevice);
		

		BowlerAbstractConnection con = newDevice.getConnection();
		Node icon = getIcon("images/connection-icon.png"
		// "images/usb-icon.png"
		);
		if (SerialConnection.class.isInstance(con)) {
			icon = getIcon(
			// "images/ethernet-icon.png"
			"images/usb-icon.png");
		} else if (UsbCDCSerialConnection.class.isInstance(con)) {
			icon = getIcon(
			// "images/ethernet-icon.png"
			"images/usb-icon.png");
		} else if (BluetoothSerialConnection.class.isInstance(con)) {
			icon = getIcon(
			// "images/ethernet-icon.png"
			"images/bluetooth-icon.png");
		} else if (UDPBowlerConnection.class.isInstance(con)
				|| BowlerTCPClient.class.isInstance(con)) {
			icon = getIcon(
			// "images/ethernet-icon.png"
			"images/ethernet-icon.png");
		}
		String line = String.format(formatStr, 
				newDevice.getScriptingName(),
				newDevice.getClass().getSimpleName(),
				newDevice.getAddress());
		plugins.add(new PluginManagerWidget(mp,icon));
		

		
		
		mp.setName(newDevice.getScriptingName());

		newDevice.addConnectionEventListener(
				new IDeviceConnectionEventListener() {
					@Override
					public void onDisconnect(BowlerAbstractDevice source) {
						// clean up after yourself...
						//disconectAndRemoveDevice(mp);
						
						for(int i=0;i<plugins.size();i++){
							PluginManager p=plugins.get(i).getManager();
							if(p.getDevice()==source){
								DeviceManager.remove(p.getDevice());
								return;
							}
						}
						
					}

					// ignore
					@Override
					public void onConnect(BowlerAbstractDevice source) {
					}
				});
		refreshItemTree();
		if(	getBowlerStudioController()!=null)
			BowlerStudioModularFrame.getBowlerStudioModularFrame().setSelectedTab(this);
	}
	private BowlerStudioController getBowlerStudioController() {
		// TODO Auto-generated method stub
		return BowlerStudioController.getBowlerStudio();
	}

	//this is needed because if you just remove one they all disapear
	private static void refreshItemTree(){
		FxTimer.runLater(
				Duration.ofMillis(100) ,() -> {
			Log.warning("Refreshing Tree size="+plugins.size());
			accordion.getPanes().clear();
			if(plugins.isEmpty())
				return;
			TitledPane last=null;
			for(int i=0;i<plugins.size();i++){
				 last=plugins.get(i);
				 accordion.getPanes().add(last);
			}
			

			if(!plugins.isEmpty()){
				disconnectAll.setDisable(false);
				accordion.setExpandedPane(last);
			}else{
				disconnectAll.setDisable(true);
			}
		});
	}
	
	private static void disconectAndRemoveDevice(PluginManager mp){
		System.out.println("Disconnecting " + mp.getName());
		Log.warning("Disconnecting " + mp.getName());
		if(mp.getDevice().isAvailable() || NonBowlerDevice.class.isInstance(mp))
			mp.getDevice().disconnect();	
		DeviceManager.remove(mp.getDevice());
	}

	@Override
	public void onDeviceRemoved(BowlerAbstractDevice bad) {
		Log.warning("Removing Device " + bad.getScriptingName());
		//new RuntimeException().printStackTrace();
		for(int i=0;i<plugins.size();i++){
			PluginManager p=plugins.get(i).getManager();
			if(p.getDevice()==bad){
				Log.warning("Found Device " + bad.getScriptingName());
				//new RuntimeException().printStackTrace();
				plugins.remove(i);
				refreshItemTree();
				return;
			}
		}
	}

	public static void addConnection() {
		Stage s = new Stage();
		new Thread() {
			public void run() {
				BowlerDatagram.setUseBowlerV4(true);
				BowlerConnectionMenu controller = new BowlerConnectionMenu();
				try {
					controller.start(s);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}.start();
		//DeviceManager.addConnection();
	}


	public static ConnectionManager getConnectionManager() {
		return connectionManager;
	}

	 public static void onFirmata() {
			Set<String> ports = NRSerialPort.getAvailableSerialPorts();
			List<String> choices = new ArrayList<>();
			if(ports.isEmpty())
				return;
			for (String s: ports){
				choices.add(s);
			}

			
			ChoiceDialog<String> dialog = new ChoiceDialog<>(choices.get(0), choices);
			dialog.setTitle("Firmata Device Serial Port Chooser");
			dialog.setHeaderText("Supports Firmata");
			dialog.setContentText("Firmata Device Port:");

			// Traditional way to get the response value.
			Optional<String> result = dialog.showAndWait();
			
			// The Java 8 way to get the response value (with lambda expression).
			result.ifPresent(letter -> {
				new Thread(()->{
					System.out.print("\nConnecting Firmata...");
					FirmataBowler p = new FirmataBowler(letter);
					p.connect();
					String name = "firmata";
					addConnection(p,name);
					System.out.print("Done!\n");
				}).start();
			});
			
		}

	
}
