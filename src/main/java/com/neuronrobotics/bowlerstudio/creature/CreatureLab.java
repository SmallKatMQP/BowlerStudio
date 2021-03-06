package com.neuronrobotics.bowlerstudio.creature;

import com.neuronrobotics.bowlerstudio.BowlerStudio;
import com.neuronrobotics.bowlerstudio.BowlerStudioController;
import com.neuronrobotics.bowlerstudio.BowlerStudioModularFrame;
import com.neuronrobotics.bowlerstudio.assets.AssetFactory;
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine;
import com.neuronrobotics.bowlerstudio.tabs.AbstractBowlerStudioTab;
import com.neuronrobotics.bowlerstudio.util.FileWatchDeviceWrapper;
import com.neuronrobotics.sdk.addons.gamepad.BowlerJInputDevice;
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine;
import com.neuronrobotics.sdk.addons.kinematics.MobileBase;
import com.neuronrobotics.sdk.common.BowlerAbstractDevice;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Parent;
import javafx.scene.control.*;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.HBox;
import org.eclipse.jgit.api.errors.GitAPIException;
import org.eclipse.jgit.api.errors.InvalidRemoteException;
import org.eclipse.jgit.api.errors.TransportException;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;

public class CreatureLab extends AbstractBowlerStudioTab implements IOnEngineeringUnitsChange {

  private BowlerAbstractDevice pm;

  private IDriveEngine defaultDriveEngine;
  // private DhInverseSolver defaultDHSolver;
  private Menu localMenue;
  private ProgressIndicator pi;

  private MobileBaseCadManager baseManager;
  private CheckBox autoRegen = new CheckBox("Auto-Regnerate CAD");

  Parent root;
  private BowlerJInputDevice gameController = null;
  CreatureLabControlsTab tab = new CreatureLabControlsTab();;

  @Override
  public void onTabClosing() {
    baseManager.onTabClosing();
  }

  @Override
  public String[] getMyNameSpaces() {
    // TODO Auto-generated method stub
    return new String[0];
  }

  @SuppressWarnings({"restriction", "restriction"})
  @Override
  public void initializeUI(BowlerAbstractDevice pm) {
    setGraphic(AssetFactory.loadIcon("CreatureLab-Tab.png"));
    this.pm = pm;
    autoRegen.setSelected(true);
    autoRegen.setOnAction(event -> {
      baseManager.setAutoRegen(autoRegen.isSelected());
      if (autoRegen.isSelected()) {
        generateCad();
      }
    });
    // TODO Auto-generated method stub
    setText(pm.getScriptingName());

    try {
      ScriptingEngine.setAutoupdate(true);
    } catch (IOException e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
    }
    MobileBase device = (MobileBase) pm;

    // Button save = new Button("Save Configuration");

    

    FXMLLoader loader;
    try {
      loader = AssetFactory.loadLayout("layout/CreatureLabControlsTab.fxml", true);
      Platform.runLater(() -> {
        loader.setController(tab);
        // This is needed when loading on MAC
        loader.setClassLoader(getClass().getClassLoader());
        try {
          root = loader.load();
          finishLoading(device);
        } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      });

    } catch (Exception e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
    }

  }

  private void finishLoading(MobileBase device) {

    TreeView<String> tree = null;
    TreeItem<String> rootItem = null;


    try {
      rootItem =
          new TreeItem<String>(device.getScriptingName(), AssetFactory.loadIcon("creature.png"));
    } catch (Exception e) {
      rootItem = new TreeItem<String>(device.getScriptingName());
    }
    tree = new TreeView<>(rootItem);
    AnchorPane treebox = tab.getTreeBox();
    treebox.getChildren().clear();
    treebox.getChildren().add(tree);
    AnchorPane.setTopAnchor(tree, 0.0);
    AnchorPane.setLeftAnchor(tree, 0.0);
    AnchorPane.setRightAnchor(tree, 0.0);
    AnchorPane.setBottomAnchor(tree, 0.0);

    rootItem.setExpanded(true);
    HashMap<TreeItem<String>, Runnable> callbackMapForTreeitems = new HashMap<>();
    HashMap<TreeItem<String>, Group> widgetMapForTreeitems = new HashMap<>();

    try {
      MobleBaseMenueFactory.load(device, tree, rootItem, callbackMapForTreeitems,
          widgetMapForTreeitems, this);
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    tree.setPrefWidth(325);
    tree.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);
    JogWidget walkWidget = new JogWidget(device);
    tree.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<Object>() {

      @Override
      public void changed(ObservableValue<?> observable, Object oldValue, Object newValue) {
        @SuppressWarnings("unchecked")
        TreeItem<String> treeItem = (TreeItem<String>) newValue;
        new Thread() {
          public void run() {
            if (walkWidget.getGameController() != null)
              setGameController(walkWidget.getGameController());
            if (callbackMapForTreeitems.get(treeItem) != null) {
              callbackMapForTreeitems.get(treeItem).run();
            }
            if (widgetMapForTreeitems.get(treeItem) != null) {

              Platform.runLater(() -> {
                tab.getControlsBox().getChildren().clear();
                Group g = widgetMapForTreeitems.get(treeItem);
                tab.getControlsBox().getChildren().add(g);
                AnchorPane.setTopAnchor(g, 0.0);
                AnchorPane.setLeftAnchor(g, 0.0);
                AnchorPane.setRightAnchor(g, 0.0);
                AnchorPane.setBottomAnchor(g, 0.0);
              });
            } else {
              Platform.runLater(() -> {
                tab.getControlsBox().getChildren().clear();
              });
              BowlerStudio.select(device);
              walkWidget.setGameController(getController());
            }
          }
        }.start();

      }
    });

    HBox progress = new HBox(10);
    pi = new ProgressIndicator(0);
    baseManager = new MobileBaseCadManager(device, BowlerStudioController.getMobileBaseUI());
    pi.progressProperty().bindBidirectional(baseManager.getProcesIndictor());

    progress.getChildren().addAll(new Label("Cad Progress:"), pi, autoRegen);

    progress.setStyle("-fx-background-color: #FFFFFF;");
    progress.setOpacity(.7);
    progress.setPrefSize(325, 50);
    tab.setOverlayTop(progress);
    tab.setOverlayTopRight(walkWidget);

    BowlerStudioModularFrame.getBowlerStudioModularFrame().showCreatureLab();

    generateCad();

    setContent(root);

  }


  public void generateCad() {
    // new Exception().printStackTrace();
    baseManager.generateCad();
  }

  @Override
  public void onTabReOpening() {
    baseManager.setCadScript(baseManager.getCadScript());
    try {
      if (autoRegen.isSelected())
        generateCad();
    } catch (Exception ex) {

    }
  }

  public static String getFormatted(double value) {
    return String.format("%4.3f%n", (double) value);
  }

  @Override
  public void onSliderMoving(EngineeringUnitsSliderWidget source, double newAngleDegrees) {
    // TODO Auto-generated method stub

  }

  @Override
  public void onSliderDoneMoving(EngineeringUnitsSliderWidget source, double newAngleDegrees) {
    if (autoRegen.isSelected())
      generateCad();
  }

  public BowlerJInputDevice getController() {

    return getGameController();
  }



  public BowlerJInputDevice getGameController() {
    return gameController;
  }

  public void setGameController(BowlerJInputDevice bowlerJInputDevice) {
    this.gameController = bowlerJInputDevice;
  }

  public void setGitDhEngine(String gitsId, String file, DHParameterKinematics dh) {
    MobileBaseLoader.get(baseManager.getMobileBase()).setDefaultDhParameterKinematics(dh);

  }

  public void setGitCadEngine(String gitsId, String file, MobileBase device)
      throws InvalidRemoteException, TransportException, GitAPIException, IOException {
    baseManager.setGitCadEngine(gitsId, file, device);
  }

  public void setGitCadEngine(String gitsId, String file, DHParameterKinematics dh)
      throws InvalidRemoteException, TransportException, GitAPIException, IOException {
    baseManager.setGitCadEngine(gitsId, file, dh);
  }


  public void setGitWalkingEngine(String git, String file, MobileBase device) {

    MobileBaseLoader.get(baseManager.getMobileBase()).setGitWalkingEngine(git, file, device);
  }


}
