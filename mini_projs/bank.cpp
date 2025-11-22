#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <string>
#include <cstdint>

class Account {
private:
    std::string name;
    u_int32_t Account_number;
    double balance;

    static int acc_count;
public:
    Account() {
        std::cout << "New blank account" << std::endl; 
        ++acc_count;
    }
    Account(std::string name_val, u_int32_t Account_number_val, double balance_val = 0)
        : name{name_val}, Account_number{Account_number_val}, balance{balance_val} {
            std::cout << "Account created" << std::endl;
            ++acc_count;
    }
    void set_name(std::string name);
    void set_accNum(u_int32_t num);

    std::string get_name() {return name;}
    u_int32_t get_AccNum() {return Account_number;}
    double get_balance() {return balance;}

    bool withdraw(double amount);
    bool deposit(double amount);

    void get_details();

    static int get_Acc_count();

    ~Account() {
        std::cout << "destructor called! Account deleted" << std::endl;
        --acc_count;
    }
};

void Account::get_details() {
    std::cout << "Name           : " << Account::get_name() << std::endl;
    std::cout << "Account Number : " << Account::get_AccNum() << std::endl;
    std::cout << "Balance        : " << Account::get_balance() << std::endl;
}

int Account::acc_count = 0;
int Account::get_Acc_count() {return acc_count;}

void Account::set_name(std::string name) {
    this->name = name;
    std::cout << "Name set!" << std::endl;
}

void Account::set_accNum(u_int32_t num) {
    Account_number = num;
    std::cout << "Account number set!" << std::endl;
}

bool Account::withdraw(double amount) {
    try {
        if (amount <= 0) {
            throw std::invalid_argument("Error withdrawing the amount");
        } else { 
            balance -= amount;
            return true;
        }
    }
    catch (const std::invalid_argument& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }
}
bool Account::deposit(double amount) {
    try {
        if (amount < 0) {
            throw std::invalid_argument("Cannot deposit a negative number");
        } else { 
            balance += amount;
            std::cout << "Withdrawin successfully" << std::endl;
            return true;
        }
    }
    catch (const std::invalid_argument& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }
}

Account* create_acc() {
    std::string name{};
    u_int32_t accNum{};
    double dep_amount{};
        
    std::cout << "Name: ";
    std::cin >> name;

    std::cout << "Account: ";
    std::cin >> accNum;

    std::cout << "Balance: ";
    std::cin >> dep_amount;

    Account* acc = new Account(name, accNum, dep_amount);
    acc->get_details();
    return acc;
}

std::vector<Account> create_multiple() {
    size_t n{};
    std::cout << "How many account would you like to make? : ";
    std::cin >> n;
    std::cout << std::endl;

    std::vector<Account> acc_vec(n);
    for (auto& i : acc_vec) {
        std::string name{};
        u_int32_t accNum{};
        double dep_amount{};
        
        std::cout << "Name: ";
        std::cin >> name;
        i.set_name(name);

        std::cout << "Account: ";
        std::cin >> accNum;
        i.set_accNum(accNum);

        std::cout << "Balance: ";
        std::cin >> dep_amount;
        i.deposit(dep_amount);
        
        i.get_details();
    }
    return acc_vec;
}

int main() {
    bool loop = true;
    int ans{};
    Account* new_acc{nullptr};
    std::vector<Account> vec_acc;

    while (loop) {
        std::cout << "Bank Services: " << std::endl;
        std::cout << "1) Create an account" << std::endl;
        std::cout << "2) Create multiple accounts" << std::endl;
        std::cout << "3) Withdraw" << std::endl;
        std::cout << "4) exit" << std:: endl;
        
        std::cin >> ans;
        switch (ans) {
            case 1:
                new_acc = create_acc();
                break;
            case 2:
                vec_acc = create_multiple();
                break;
            case 3: 
                if (!new_acc && vec_acc.empty()) {
                    std::cout << "Accounts not created" << std::endl;
                    break;
                }
                if (new_acc) {
                    double amount{};
                    std::cout << "Withdraw amount: ";
                    std::cin >> amount;
                    std::cout << std::endl;

                    new_acc->withdraw(amount);
                    break;
                }
                if (!vec_acc.empty()) {
                    int acc{};
                    std::cout << "From which account would you like to withdraw: ";
                    std::cin >> acc;

                    if (acc > vec_acc.size()) {
                        std::cout << "Error occured" << std::endl;
                    } else {
                        double amount{};
                        std::cout << "Withdraw amount: ";
                        std::cin >> amount;
                        std::cout << std::endl;

                        vec_acc.at(acc - 1).withdraw(amount);
                        break;
                    }
                }
            case 4:
                std::cout << "Thanks for banking with us!" << std::endl;
                if (!new_acc && vec_acc.empty()) {
                    loop = false; 
                    break;
                }
                if (new_acc) {
                    new_acc->get_details();
                    loop = false;
                    break;
                } else if (!vec_acc.empty()) {
                    for (auto& acc : vec_acc) {
                        acc.get_details();
                        std::cout << std::endl;
                        loop = false;
                        break;
                    }
                }
            default:
                std::cout << "Invalid option!" << std::endl;
                break;
        }
    }
    delete new_acc;
    return 0;
}