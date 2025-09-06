simple Txc2
	{
	    parameters: // moteqayer haye lazem baraye code c++ ya dar sorati k etelate ezafi be yek module bekhahim ezafe konim dar in bakhsh in karo mikonim 
	        @display("i=block/routing"); // add a default icon -- in icon ha pishtar file png anha mojod ast
	    gates:
	        input in;
	        output out;
	}
	
	//
	// Make the two module look a bit different with colorization effect.
	// Use cyan for `tic', and yellow for `toc'.
	//
	network Tictoc2
	{
	    submodules:
	        tic: Txc2 {
	            parameters:
	                @display("i=,cyan");//gozine aval qable virgol dar vaqe haman shekl moshakhas mikone chon qablan moshakhas shode dgar niazi be taqir nist // do not change the icon (first arg of i=) just colorize it
	        }
	        toc: Txc2 {
	            parameters:
	                @display("i=,gold"); // here too
	        }
	    connections:
	        tic.out --> {  delay = 100ms; } --> toc.in;
	        tic.in <-- {  delay = 100ms; } <-- toc.out;
	}