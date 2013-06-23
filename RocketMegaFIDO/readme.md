RocketMega FIDO
===============

Ground-based FIDO software that interacts with the [RocketMega](https://github.com/zortness/rocket-mega-code/tree/master/RocketMega)
software connected via COM port.


Building
------------
This project requires the following software to build:
* [Java JDK SE 6+](http://www.oracle.com/technetwork/java/javase/downloads/index.html)
* [Ant](http://ant.apache.org/)
* [Ivy](http://ant.apache.org/ivy/)

```bash
ant jar
java -jar jar/RocketMegaFido-[version].jar
```

Once compiled, the .jar file will run as a standalone Java application.