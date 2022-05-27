/**
 * @file Singleton.hpp
 * @author Bogdan Simulj
 * @brief Implementation of the singleton class
 * @version 0.1
 * @date 2022-04-24
 *
 * @copyright Copyright (c) 2022
 *
 * General implementation of the singleton class trough inheritance method:
 *
 * class MailBox : public Singleton<MailBox>
 *   {
 *       friend class Singleton<MailBox>;
 */
#pragma once
template <typename T>
class Singleton
{
public:
    static T &Instance()
    {
        static T instance;
        return instance;
    }

protected:
    Singleton()
    {
    }

private:
    Singleton(Singleton const &);
    Singleton &operator=(Singleton const &);
};

