# opp_database

## Database Setup

1. Open the mysql client and create a database with the correct name:
    ```
    mysql -u root -p
    mysql> CREATE DATABASE opp;
    mysql> exit
    ```
    ```

2. You can test that the package & database have been set up correctly:
    ```
    catkin run_tests opp_database
    ```
