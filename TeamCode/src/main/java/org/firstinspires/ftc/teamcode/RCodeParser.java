package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;


public class RCodeParser {
    /*
     *   We call RCode file a text file with the extension *.rc that contain the RC program
     *
     *   We call a BLOCK a line of RCode Ex. : X100 Y10 F10 GC50
     *   A WORD is one of the instructions in a block Ex. : X100
     *   A FUNCTION is what the command has to do Ex. X100 function X means axis X needs to move to a certain value
     *   A SET-POINT or Function ID is a number that tells where to go or in case of H,P,Q functions tells what function number is called
     *
     *   This class reads from RCode file and decodes the current block into robot target positions, target speeds, and parameters
     *   These parameters are used in AutoOpMode
     */
    //DEBUGPC public static String filePath = "D:\\Auto.rc";//ConfigVar.rcPrgm.rcFilePath;    // File path to the RCode file ( "auto.rc" ??)
    public static String filePath = "/sdcard/FIRST/auto.txt";//ConfigVar.rcPrgm.rcFilePath;    // File path to the RCode file ( "auto.rc" ??)
    // This string contains all lines that were read from the RCode
    //private static String [] rcBlocks;
    static ArrayList<String> rcBlocks;
    // Current BLOCK that is parsed
    private static int execBlock = 0;
    // Number of BLOCKS in the RCode
    private static int noBlocks = 0;
    private static   HashMap<String,String>     rcVariables;
    private static  HashMap<String,String>      rcGFunct;
    // Library of WORDS in the RCode
    private String [] wLibrary = {"#","$","X","Y","R","F","Z","S","P","Q","M" ,"G","/", "SK","M"};
    public HashMap<String,String> rcSetPoints;

    // Library of M functions in the RCode
    String [] mLibrary = {"M1","M2","M3","M4","M5","M6","M7","M8","M9"};
    // Error codes
    enum RCodeErrors {rcErrFileNotFound, rcErrSyntax, rcErrVariable, rcErrFunction, rcErrBlock, rcErrUnknown, rcNoError, rcErrFileRead, rcErrException};
    // Error code
    static private RCodeErrors rcError = RCodeErrors.rcNoError;
    static int exeSts=0;
    // Number of WORD commands in the library
    public static int LibCount;
    // Number of WORDS in a BLOCK
    public static int WORDS_COUNT = 19;
    // Status of the RCode program
    enum RCPrgStatus {rcNotLoaded, rcLoaded, rcRunning, rcHolding, rcEnded};
    // Status of the RCode program
    static RCPrgStatus rcStatus;
    static boolean blockSkip = false;

    boolean modalMecanum = false;
    boolean modalServos = false;
    boolean modalSlider = false;
    boolean [] modalPCodes = {false,false,false,false,false};
    boolean [] modalQCodes = {false,false,false,false,false};
    boolean     modalM2 = false;

    // Return the WORDS array of the current BLOCK
    HashMap<String, String> getCommands( ){ return rcSetPoints; }
    // Returns R-Code status
    RCPrgStatus getStatus(){ return rcStatus; }


    /*
     *   Test main method to decode the RCode file
     */

    // Constructor
    public RCodeParser()
    {
        rcBlocks = new ArrayList<>();
        rcVariables = new HashMap<>();
        rcGFunct = new HashMap<>();
        rcSetPoints = new HashMap<>();

        rcStatus = RCPrgStatus.rcNotLoaded;
        execBlock = 0;
    }

    // Method used to read the entire file content into an array of String as BLOCKS
    public boolean loadFile()
    {
        File rcPrgmFile = new File(filePath);
        String crtBlock;
        if(  rcStatus != RCPrgStatus.rcNotLoaded )
        {
            exeSts = 1;
            return false;
        }
        // Check if the RCode File file exists and there are permissions
        //"Error: File does not exist at path: " + filePath;
        if( !rcPrgmFile.exists() )
        {
            exeSts = 2;
            rcError = RCodeErrors.rcErrFileNotFound;
            System.out.println(".exists error");
            return false;
        }
        // "Error: File cannot be read. Check permissions for: " + filePath;
        if( !rcPrgmFile.canRead() ){ rcError = RCodeErrors.rcErrFileRead; System.out.println(".read error"); return false; }
        exeSts = 3;
        try (BufferedReader reader = new BufferedReader(new FileReader(rcPrgmFile)))
        {
            exeSts = 4;
            while ((crtBlock = reader.readLine()) != null )
            {
                exeSts++;
                if(! crtBlock.startsWith("//") && !(crtBlock.trim().isEmpty()))  rcBlocks.add( crtBlock );
                // If  not a comment in RCode add to Blocks list
            }
            noBlocks = rcBlocks.size();
            rcStatus = RCPrgStatus.rcLoaded;
            execBlock = 0;
            exeSts = 500;
        } catch (Exception ex) {
            // Handle unexpected errors gracefully
            exeSts = 700;
            rcStatus = RCPrgStatus.rcNotLoaded;
            rcError = RCodeErrors.rcErrException;
            System.out.println(".exception error file");
            return false; //"Error: An unexpected error occurred while reading the file.";
        }
        return true;
    };
    public String crtBlock;
    // Parse a line of RCode
    boolean rcParseLine( int lineNo )
    {
        // If Block is empty return false
        if (rcBlocks.get(lineNo) == null) return false;
        // Get the current block
        //String  crtBlock;
        crtBlock = rcBlocks.get(lineNo);
        // Keeps the splitted block into words
        String []   blockParts;
        String      outPrint = "";

        // Index for the WORDS
        int iWord = 0;
        rcSetPoints.clear();

        // If the blockSkip is active parseSkipBlock return "NOP" indicating that the block should be skipped
        crtBlock = parseSkipBlock(crtBlock);
        if( ( crtBlock.equals("NOP")) ) return true;
        // If the block starts with # it is a variable declaration
        // Ex: #SliderSpeed 10
        if( preParseVarDeclaration(crtBlock)) return true;
        // Check for errors in previous parsing
        if( rcError != RCodeErrors.rcNoError ) return false;
        // If not a variable declaration split the block into words
        blockParts = crtBlock.split(" ");
        System.out.print("L" + Integer.toString(lineNo) + ": ");
        // The below wile returns the set-points of the current block
        iWord = 0;
        while( iWord < blockParts.length )
        {
            if( !checkWordSyntax(blockParts[iWord]))
            { rcError = RCodeErrors.rcErrVariable;
                System.out.println("Syntax Error");
                return false;
            }
            // If the word starts with G it is a function and requires reformatting of the block
            blockParts[iWord] = preParseGFunction(blockParts[iWord]);

            // Checks which command is it using wLibrary
            for(int iLib = 2 /*without '#' */; iLib < wLibrary.length   ; iLib++)
            {
                // Check which WORD is it
                if( blockParts[iWord].startsWith(wLibrary[iLib] ) )
                {
                    outPrint = parseSetPoint(blockParts[iWord]);
                    System.out.print(wLibrary[iLib]); System.out.print( rcSetPoints.get(outPrint) ); System.out.print(" ");
                    break;
                }
            }
            iWord++;
        }
        System.out.println("");
        return true;
    }

    public String cleanUpBlock(String blockParts)
    {
        String retBlock = blockParts;
        // Clean up the block of spaces and commas
        retBlock = retBlock.toUpperCase();
        retBlock = retBlock.replaceAll("[\\t, ]",""); // trim front
        return retBlock;
    }

    private String parseSkipBlock(String blockPart)
    {
        // If blockSkip is true /X100 Y100 -> X100 Y100
        // If blockSkip is false returns "NOP"

        if( blockPart.startsWith( "SK") )
        {
            blockSkip = true; // If the lock starts with "SK" enable blockSkip command
            return "NOP";
        }
        if( blockPart.startsWith( "/") )
        {
            if( blockSkip )
            {
                System.out.println("Skip block");
                return "NOP";
            }
            // remove Skip symbol from the front of the block
            blockPart = blockPart.substring(1);
            blockPart = blockPart.trim();
            return blockPart;
        }
        return blockPart;
    }

    // Parse a variable in a function G$Gripper#gripperOpen
    String parseSetPoint(String blockPart)
    {
        // Split the code line using "#" as the delimiter
        String[] parts;
        String tmpPart;
        String regex;
        blockPart = cleanUpBlock(blockPart);
        // Check if the block contains $ and #
        if(blockPart.matches(".*[#$].*"))
        {
            // Check if the variable is a G1$150 format
            if( blockPart.contains("$"))
            {
                // Format is G1$150 -> G1 150
                parts = blockPart.split("$");
                // parts[1] - contains setpoint value
                rcSetPoints.put(parts[0], parts[1]);
                return parts[0];
            }
            if( blockPart.contains("#") )
            {
                // Format is G1#GripperOpen or M#mFunct -> G1 GripperOpen M 1
                parts = blockPart.split("#");
                //if( ! rcGFunct.containsKey(parts[1]) ){ rcError = RCodeErrors.rcErrVariable; return null; }
                if( parts[0] == null || parts[1] == null ){ rcError = RCodeErrors.rcErrVariable; return null; }
                parts[1] = rcVariables.get( parts[1] ); // GripperOpen -> 1 ( declared as var #gripperOpen 1 )
                // Different parsing based on kind of command words
                tmpPart = parts[0].substring(0,1);  // Separate 1st leter from the command word
                // Check if command exists in the R-Code library
                if( !checkLibrary( parts[0].substring(0,1) ) ){ rcError = RCodeErrors.rcErrVariable; return null; }
                switch( tmpPart )
                {
                    case "P": case "Q": case "M":
                    // Add to parts[0] the function number M -> M1
                    parts[0] = parts[0] + parts[1];   // Makes parts[0] M -> M1
                    break;
                    case "G": // G already in good format as G1
                        break;
                    default: // Rest of words: X,Y,R...
                // These are also in good format now {X 100}
                }
                // All parts in good format now : {G1, 1} or {M2, 2}, or {X, 120} ...
                rcSetPoints.put(parts[0],parts[1]);
                return parts[0];
            }
        }else
        {
            // blockPart does not contains $ or #
            // Format is M1, P2, X150.2, ...
            tmpPart = blockPart.substring(0,1);
            parts = blockPart.split("(?<=^[A-Z])");//("(?<=\\D)(?=\\d)");
            if( parts[0] == null || parts[1] == null ){ rcError = RCodeErrors.rcErrVariable; return null; }
            switch( tmpPart )
            {
                case "P": case "Q": case "M":
                // Add to parts[0] the function number M -> M1
                parts[0] = blockPart;   // Makes parts[0] M -> M1
                break;
                case "G": // G should not be here it can only be called with G$150 or G#gripperOpen
                    rcError = RCodeErrors.rcErrVariable; return null;
                default: // Rest of words: X,Y,R,P,M, ...
            }
            // These are also in good format now {X, 100}
            rcSetPoints.put(parts[0], parts[1]);
            return parts[0];
        }
        return null;
    }

    private boolean preParseVarDeclaration(String blockPart )
    {
        /*
        A variable declaration is a line that starts with "#"
        #poleIdle 10
        The method will extract the variable name and the value and add it to the rcVariables HashMap
        */
        String regex;
        boolean varDecl = false;
        // The block do not starts with # or $
        if( !( blockPart.startsWith( "#") || blockPart.startsWith( "$")) ) return false;
        // It's a variable or G function declaration
        varDecl =  ( blockPart.startsWith( "#") );
        blockPart = cleanUpBlock(blockPart);
        // Ex: #SliderSpeed 10 -> SliderSpeed 10
        blockPart = blockPart.substring(1);
        // Split the code line into alpha and numeric
        regex = "(?<=\\D)(?=\\d)";
        String[] parts = blockPart.split(regex);

        if( parts[0] == null || parts[1] == null ){ rcError = RCodeErrors.rcErrVariable; return false; }
        // Add variable to HashMap rcVariables
        if( varDecl ) rcVariables.put(parts[0],parts[1]);
            // Add G Identifier to HashMap rcGFunct
        else rcGFunct.put(parts[0], parts[1]);
        System.out.print(parts[0]); System.out.print(": "); System.out.println(parts[1]);
        return true;
    }

    // Checks if wordPart is an existing xommand word
    boolean checkLibrary( String wordPart)
    {
        for (int i = 0; i < wLibrary.length; i++)
            if( wordPart.equals( wLibrary[i] ) ) return  true;
        return false;
    }

    String preParseGFunction(String blockPart )
    {
        /*
        A G variable declaration is a line that starts with "$"
        $gripperArm 1
        The method will extract the variable name and the value and add it to the rcGFunc HashMap
        */
        String [] parts;
        if( blockPart.startsWith( "G" ))
        {
            blockPart = cleanUpBlock(blockPart);

            if(blockPart.contains("#"))
            {
                // Split the code line format: G$Gripper#GripperOpen into G1#GripperOpen
                parts = blockPart.split("[$#]");
                if( parts[0] == null || parts[1] == null || parts[2] == null ){ rcError = RCodeErrors.rcErrVariable; return "NOP"; }
                blockPart = "G" + rcGFunct.get(parts[1]) + "#" + parts[2];
                return blockPart;
            }else
            {   // Split the code line format: G$Gripper120 into G1$120
                //regex = "([A-Z])\\$(\\w+)(.*)";
                parts = blockPart.split("(?<=\\$)|(?<=[a-zA-Z])(?=\\d)");
//                parts = blockPart.split(regex);
                if( parts[0] == null || parts[1] == null || parts[2] == null ){ rcError = RCodeErrors.rcErrVariable; return "NOP"; }
                blockPart = "G" + rcGFunct.get(parts[1]) + "$" + parts[2];
                return blockPart;
            }
        }
        return blockPart;
    }

    boolean checkWordSyntax( String blockPart)
    {
        String tmpPart = blockPart.replaceAll("[^A-Z].*", "");
        for (int i = 0; i < wLibrary.length; i++)
        {
            if( tmpPart.equals(wLibrary[i]) || blockPart.matches("^[/#$]")) return true;
        }
        return false;
    }

    //  Return an array with the active WORDS in the next BLOCK of the RCode
    //  the return String rcWords is the list that contains set-points of all Words from this block
    //  For example for the block:
    //  X25 Y50 F10 GT20
    //  The function would return:
    public boolean  nextBlock()
    {
        String [] rcWords = new String[WORDS_COUNT];
        // If the program is loaded and running we can parse the next BLOCK
        if(( rcStatus == RCPrgStatus.rcLoaded || rcStatus == RCPrgStatus.rcRunning) &&  execBlock < noBlocks )
        {
            exeSts = 6;
            if( !rcParseLine(execBlock)) return false;
            execBlock++;
        }
        if( execBlock == noBlocks )
        {
            execBlock=0; rcStatus = RCPrgStatus.rcEnded;
        }
        if( rcSetPoints.get("M99") != null && rcSetPoints.get("M99").equals("99") )
            rcStatus = RCPrgStatus.rcEnded;
        return true;
    }

    // Returns SetPoint Value of a command word
    String getTarget( String wFunc )
    {
        String retString = (wFunc == "null" )?"NOP":wFunc;
        String retValue = rcSetPoints.get( retString );
        return ( retValue != null)? retValue : "NOP" ;
    }

    boolean blockActive( String wFunc )
    {
        String retString = (wFunc == "null" )?"NOP":wFunc;
        String retValue = rcSetPoints.get( retString );
        return ( retValue != null)? true : false ;
    }

    boolean anyMopdalActive()
    {
        boolean modalFlag = false;
        for (int i = 1; i < 5; i++)
        {
            if( modalPCodes[i] || modalQCodes[i] ) modalFlag = true;
        }
        if( modalM2 ) modalFlag = true;
        return modalFlag;
    }

    public  boolean mecanumModalActive( boolean modalMecanumGroup)
    {
        if( modalMecanumGroup ) modalMecanum = false;
        if( blockActive("X") || blockActive("Y") || blockActive("R"))
            if( !modalMecanumGroup ) modalMecanum = true;
        return modalMecanum;
    }

    public boolean  sliderModalActive(boolean sliderGroupReady )
    {
        if( sliderGroupReady ) modalSlider = false;
        if( blockActive("Z") )
            if( !sliderGroupReady) modalSlider = true;
        return modalSlider;
    }

    public boolean servoModalActive( boolean servoGroupReady)
    {
        String gFunc;
        if( servoGroupReady ) modalServos = false;
        for (int i = 0; i < rcGFunct.size(); i++)
        {
            gFunc = "G" + i + "";
            if( blockActive( gFunc )  )
                if( !servoGroupReady ) modalServos = true;
        }
        return modalServos;
    }

    public boolean modalFuncActive(String wFunc, boolean  sensorStatus )
    {
        if(isModalActive(wFunc) && sensorStatus ) disableModal(wFunc);
        return isModalActive(wFunc);
    }
    boolean isModalActive(String funcCode )
    {
        switch( funcCode )
        {
            case "P1": if( modalPCodes[1]) return true;
            case "P2": if( modalPCodes[2]) return true;
            case "P3": if( modalPCodes[3]) return true;
            case "P4": if( modalPCodes[4]) return true;
            case "Q1": if( modalQCodes[1]) return true;
            case "Q2": if( modalQCodes[2]) return true;
            case "Q3": if( modalQCodes[3]) return true;
            case "Q4": if( modalQCodes[4]) return true;
            case "M2": if( modalM2 ) return true;
                break;
        };
        return false;
    }

    void disableModal( String funcCode )
    {
        switch( funcCode)
        {
            case "P1": modalPCodes[1] = false; break;
            case "P2": modalPCodes[2] = false; break;
            case "P3": modalPCodes[3] = false; break;
            case "P4": modalPCodes[4] = false; break;
            case "Q1": modalQCodes[1] = false; break;
            case "Q2": modalQCodes[2] = false; break;
            case "Q3": modalQCodes[3] = false; break;
            case "Q4": modalQCodes[4] = false; break;
            case "M2": modalM2 = false; break;
        }
    }

    // Returns true when the M mNumber is in rcSetPoints
// Also enables modal status
    void parseModal( )
    {
        if( blockActive("P1")){ modalPCodes[1] = true; rcSetPoints.remove("P1"); }
        if( blockActive("P2")){ modalPCodes[2] = true; rcSetPoints.remove("P2"); }
        if( blockActive("P3")){ modalPCodes[3] = true; rcSetPoints.remove("P3"); }
        if( blockActive("P4")){ modalPCodes[4] = true; rcSetPoints.remove("P4"); }
        if( blockActive("Q1")){ modalQCodes[1] = true; rcSetPoints.remove("Q1"); }
        if( blockActive("Q2")){ modalQCodes[2] = true; rcSetPoints.remove("Q2"); }
        if( blockActive("Q3")){ modalQCodes[3] = true; rcSetPoints.remove("Q3"); }
        if( blockActive("Q4")){ modalQCodes[4] = true; rcSetPoints.remove("Q4"); }
        if( blockActive("M2")){ modalM2 = true; rcSetPoints.remove("M2"); }
    }
}