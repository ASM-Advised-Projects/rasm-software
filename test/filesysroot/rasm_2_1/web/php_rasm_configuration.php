
<?php

	$aa= $_POST["aa"];
	$ba= $_POST["ba"];
	$ca= $_POST["ca"];
	$da= $_POST["da"];
	$ea= $_POST["ea"];
	

	echo "for left-right distance offset from center: $aa<br>";
	echo "for up-down distance offset from center: $ba<br>";
	echo "Left-right angle offset from center: $ca degrees<br>";
	echo "up-down angle offset from center: $da degrees<br>";
	echo "screen to face distance: $ea degrees<br>";
	
	// save the file to text file
	$data = $aa.' '.$ba.' '.$ca.' '.$da.' '.$ea;
	$ret = file_put_contents('mydata.txt', $data, FILE_APPEND | LOCK_EX);
    if($ret === false) {
        die('There was an error writing this file');
    }
    else {
        echo "<br>$ret bytes written to file";
    }
?>
<h1>Form has been succesfully submitted</h1> 
<a href="configurations.html">Back to Configuration page</a>
