# opp_database

## Database Setup

1. Set up the `mysql` database.
    - Create a folder called `offline_generated_paths` at `~/.local/share/offline_generated_paths`
    - Install `mysql` and make sure you have root/admin access.
        1. If you do not already have `mysql` installed:
            1. Install `mysql`. Follow the directions in the [`mysql` website](https://dev.mysql.com/doc/mysql-apt-repo-quick-guide/en/#apt-repo-fresh-install) to install the debian
                > NOTE: If you set a password you will need to remember it in order to log in into a mysql session.

            1. Initialize `mysql`. Run the following command as root or with `sudo` in order to initialize `mysql`.
            We'll use the `--initialize-insecure` option at first:
                ```bash
                mysqld --initialize-insecure --user=mysql
                ```

                > Note: Remember to use `sudo` if you aren't logged in as root. For more details go to the [mysql webpage](https://dev.mysql.com/doc/refman/8.0/en/data-directory-initialization-mysqld.html)

            1. Set the password for the database
                - Connect to the server as root with no password
                    ```bash
                    mysql -u root --skip-password
                    ```
                    > NOTE: Should this failed because a password is required.

                    Next, log in as follows:

                    ```bash
                    mysql -u root -p
                    ```
                    Enter your password when asked.
                - You'll now be in a `mysql` console session. Set `0000` as the password for the root account if one hasn't been set yet.
                    > Note: You should probably use something more secure than `0000`, but what would be the fun in that?

                    ```bash
                    ALTER USER 'root'@'localhost' IDENTIFIED BY '0000';
                    ```
                - Press Ctrl-D to log out
                - Log in again using the password
                    ```bash
                    mysql -u root -p
                    ```

                > NOTE: For more info see the [mysql webpage](https://dev.mysql.com/doc/refman/8.0/en/default-privileges.html).
                > Additional help on resetting the password can be found [here](https://www.digitalocean.com/community/tutorials/how-to-import-and-export-databases-and-reset-a-root-password-in-mysql)

        1. If you do already have `mysql` installed, but you do not know the root password:
            - My favorite hack for getting into a `mysql` server with an unknown password is detailed [here](https://askubuntu.com/questions/766900/mysql-doesnt-ask-for-root-password-when-installing), in the answer by pwxcoo.

    - An *opp* database needs to be created manually.  The necessary tables will be automatically generated.
        ```bash
        $ mysql -u root -p
        mysql> CREATE DATABASE opp;
        mysql> SHOW DATABASES;
        ```
    - The *ros-client* `mysql` user needs to be created and its password should be `0000`.  It only needs permission to the opp database created above.  If you want it to have all permissions, you can use the command below.
        ```bash
        $ mysql -u root -p
        mysql> CREATE USER 'ros-client'@'localhost' IDENTIFIED BY '0000';
        mysql> GRANT ALL PRIVILEGES ON opp.* TO 'ros-client'@'localhost' WITH GRANT OPTION;
        ```
        > NOTE: This example uses '0000' for the password. You could use a safer password, but this one is currently hardcoded.
        > See [here](https://dev.mysql.com/doc/refman/8.0/en/adding-users.html) for more info on user privileges.

1. Open the `mysql` client and create a database with the correct name:
    ```bash
    mysql -u root -p
    mysql> CREATE DATABASE opp;
    mysql> exit
    ```
1. You can test that the package and database have been set up correctly:
    ```bash
    catkin run_tests opp_database
    ```
