
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@TeleOp(name="Hdrive", group="Iterative Opmode")
@Disabled

public class Hdrive extends OpMode
{
    
    DcMotor frontleftmotor = null; 
    DcMotor frontrightmotor = null; 
    DcMotor backleftmotor = null;
    DcMotor backrightmotor = null; 
    DcMotor middlemotor = null; 
    @Override
    public void init() {
        
    frontleftmotor = hardwareMap.get(DcMotor.class, "flm");
    frontrightmotor = hardwareMap.get(DcMotor.class, "frm");
    backleftmotor = hardwareMap.get(DcMotor.class, "blm"); 
    backrightmotor = hardwareMap.get(DcMotor.class, "brm");
    middlemotor = hardwareMap.get(DcMotor.class, "mm");
    
    frontleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    middlemotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
       
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    double x = 0;
    double y = 0;
    double rotation = 0;
    x = gamepad1.left_stick_x;
    y = gamepad1.left_stick_y*-1;
    rotation = gamepad1.right_stick_x;
    double frontleftpower=0;
    double frontrightpower=0;
    double backleftpower=0;
    double backrightpower=0;
    double middlemotorpower = 0;
    
    
    middlemotorpower = middlemotorpower+x; 
    
    
    frontleftpower = frontleftpower+y;
    frontrightpower = frontrightpower+y;
    backleftpower = backleftpower+y;
    backrightpower = backrightpower+y;
    
    
    frontleftpower = frontleftpower + rotation; 
    frontrightpower = frontrightpower - rotation; 
    backleftpower = backleftpower + rotation; 
    backrightpower = backrightpower - rotation;
    
     
    frontleftmotor.setPower(frontleftpower);
    frontrightmotor.setPower(frontrightpower);
    backleftmotor.setPower(backleftpower);
    backrightmotor.setPower(backrightpower);
    middlemotor.setPower(middlemotorpower);
    
    
    
    
    
    
    
    }   
    @Override
    public void stop() {
    }


}
