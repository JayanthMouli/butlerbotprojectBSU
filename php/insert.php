<!--                         JAYANTH MOULI
                            BOISE HIGH SCHOOL
                            BSU INTERN 2018-19
                            
.^o ~\
Y /'~) }      _____
l/  / /    ,-~     ~~--.,_
   ( (    /  ~-._         ^.
    \ "--'--.    "-._       \
     "-.________     ~--.,__ ^.
               "~r-.,___.-'-. ^.
                YI    \      ~-.\
                ||     \        `\
                ||     //
                ||    //
                ()   //
                ||  //     
                || ( c
   ___._ __  ___I|__`--__._ __  _
 "~     ~  "~   ::  ~~"    ~  ~~
                ::
                .:
                 .
-->


<?php
    if (isset($_GET['destination'])) {
    $con = mysqli_connect("db4free.net", "ece218remote", "butlercall", "butlerbot_destdb");
	if(mysqli_connect_errno($con)) {
	echo "Failed to connect";
	}	
	$input = $_GET['destination'];
	$query = "INSERT INTO dest_values(goto) VALUES ('$input')";
	if ($con->query($query) === TRUE) {
    echo "New record created successfully";
    } 
    else {
    echo "Error: " . $query . "<br>" . $con->error;
    }
    }
    else {
        echo "no specified destination";
    }
    mysqli_close($con);
    

?>

