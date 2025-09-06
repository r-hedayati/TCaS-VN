//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
simple Txc4 {//mikhahim parameter hayi be onvane aqaz konnade ersal va yek parametr k limit tedad ersal ra moshakhas kone ==> pas ham mishe in kar ro tanha dar c++ kard ham be onvane parametr

	parameters:
    
    	bool sendMsgOnInit = default(false); // moteqayer baraye shore ersal dar hengame init
    	int limit = default(2);//tavajoh shavad meqdare moteqayerha mitavand dar .ini moshakhas shavad
    	@display("i=block/routing");
    
    gates:
        
        input in;
        output out;
       
}
network Tictoc4 {

submodules:
    tic: Txc4 {
        parameters:
            sendMsgOnInit=true; // inja moshakhas mikonim k tic aval befreste ya toc
    		@display("i=,cyan");
    }
    toc: Txc4 {
    	parameters:
    	    sendMsgOnInit = false;
    	    @dsiplay("i=,gold");
    }
    
    connections:
        tic.out--> {delay=100ms;} --> toc.in;
        tic.in <-- {delay=100ms;} <-- toc.out;
    }

