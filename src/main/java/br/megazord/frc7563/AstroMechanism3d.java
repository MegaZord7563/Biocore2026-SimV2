package br.megazord.frc7563;

public class AstroMechanism3d {
    private static AstroMechanism3d instance;
    
    public static AstroMechanism3d getInstance() {
    if (instance == null) {
      instance = new AstroMechanism3d();
    }
    return instance;
  }
}
